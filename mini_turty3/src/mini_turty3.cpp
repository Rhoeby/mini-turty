#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

#include "pigpiod_if2.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

/*
NOTE - Must start 'pigpiod' daemon first:
  sudo pigpiod
*/

// options
#define MINI_USE_CONSOLE 1

// GPIO pin assignments
//#define GPIO_SLEEP 16
#define GPIO_SLEEP 13
#define GPIO_DIR0  17
#define GPIO_DIR1  26
#define GPIO_STEP0 18
#define GPIO_STEP1 19

#define MOTOR_SPEED_SLEW_SIZE 0.1 // rate of change of motor speed (in m/s)

typedef enum DIFFDRIVE_MOTOR
{
  DIFFDRIVE_MOTOR_LEFT,
  DIFFDRIVE_MOTOR_RIGHT,
  DIFFDRIVE_MOTOR_MAX
} DIFFDRIVE_MOTOR;

typedef struct Motor_t {
  double currentSpeed;
  double targetSpeed;
} Motor_t;

class MiniTurtyBase
{
  public:
    MiniTurtyBase(int the_pi);
    virtual ~MiniTurtyBase() {};

    void setEnableOdomTransform(const bool enabled);
    void setCmdVelTopic(const std::string topic_name);
    void setMotorSpeed(DIFFDRIVE_MOTOR motor_num, double speed);

  protected:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist);
    void timerCallback(const ros::TimerEvent&);
    void updateMotorControl(DIFFDRIVE_MOTOR motor_num);
    void publishOdom(const double vx, const double vth);

  private:
    int pi;
    ros::NodeHandle node;
    ros::Subscriber cmd_vel_sub;
    ros::Timer timer;
    Motor_t motors[DIFFDRIVE_MOTOR_MAX];
    tf::TransformBroadcaster odom_broadcaster;
    bool odomTransformEnabled;
};

int init_gpio(void);

#if MINI_USE_CONSOLE
void consoleSetTerminalMode(void);
char processConsole(MiniTurtyBase *mini);
#else
void sig_handler(int s);
static bool got_ctrl_c = false;
#endif

//////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  int pi;
  
  printf("Hello!\n");

  pi = init_gpio();
  if(pi) {
    printf("Error initializing pigpio!\n");
    return pi;
  }

  ros::init(argc, argv, "mini_turty");

#if MINI_USE_CONSOLE
  consoleSetTerminalMode();
#else
  signal(SIGINT, sig_handler);
#endif

  ros::NodeHandle private_node_handle_("~");
  ros::Rate loop_rate(10);

// JJ - provide odom transform
#if 1
  int enable_odom_transform;

//  private_node_handle_.param("enable_odom_transform", enable_odom_transform, int(0));
  private_node_handle_.param("enable_odom_transform", enable_odom_transform, int(1));
#endif

  MiniTurtyBase mini(pi);
  mini.setCmdVelTopic("cmd_vel");
// JJ - provide odom transform
#if 1
  mini.setEnableOdomTransform(enable_odom_transform);
#endif

  while (ros::ok()) 
  {
#if MINI_USE_CONSOLE
    if(processConsole(&mini) == 'q') {
      ROS_WARN("Got 'q'");
      break;
    }
#else
    if (got_ctrl_c) {
      ROS_WARN("Got Ctrl-C");
      break;
    }
#endif

    ros::spinOnce();
    loop_rate.sleep();
  }

  // set SLEEP (active low)
  gpio_write(pi, GPIO_SLEEP, 0);
  
  pigpio_stop(pi);

  ROS_WARN("Exiting.");

  exit(0);
}

/////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------
 *
 *---------------------------------------------------------------------*/
MiniTurtyBase::MiniTurtyBase(int the_pi)
{
  pi = the_pi;

  motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed = 0;
  motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed = 0;
  motors[DIFFDRIVE_MOTOR_LEFT].targetSpeed = 0;
  motors[DIFFDRIVE_MOTOR_RIGHT].targetSpeed = 0;

  timer = node.createTimer(ros::Duration(0.05), &MiniTurtyBase::timerCallback, this);
}

/*---------------------------------------------------------------------
 *
 *---------------------------------------------------------------------*/
void MiniTurtyBase::timerCallback(const ros::TimerEvent&)
{
  double vx;
  double vth;
  
  updateMotorControl(DIFFDRIVE_MOTOR_LEFT);
  updateMotorControl(DIFFDRIVE_MOTOR_RIGHT);

  vx = (motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed + 
        motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed) / 2;
//  vth = (motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed - 
//         motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed) * 8.25;
//  vth = (motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed - 
//         motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed) * 4;
  vth = (motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed - 
         motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed) * 8;

  // TODO: add a second param to control this rather than hijacking odom *transform* enable
  // don't publish odom when running hector!
  if(odomTransformEnabled)
    publishOdom(vx, vth);
}

/*---------------------------------------------------------------------
 *
 *---------------------------------------------------------------------*/
void MiniTurtyBase::updateMotorControl(DIFFDRIVE_MOTOR motor_num)
{
  double currentSpeed = motors[motor_num].currentSpeed;
  double targetSpeed = motors[motor_num].targetSpeed;
  
  if(currentSpeed == targetSpeed) return;

  // slew the current speed toward the target speed
  if(currentSpeed < targetSpeed) {
    if(currentSpeed+MOTOR_SPEED_SLEW_SIZE > targetSpeed)
      currentSpeed = targetSpeed;
    else
      currentSpeed += MOTOR_SPEED_SLEW_SIZE;
  }
  else if (currentSpeed > targetSpeed) {
    if(currentSpeed-MOTOR_SPEED_SLEW_SIZE < targetSpeed)
      currentSpeed = targetSpeed;
    else
      currentSpeed -= MOTOR_SPEED_SLEW_SIZE;
  }

  // write the current speed and direction to the motor control
  switch(motor_num) 
  {
  case DIFFDRIVE_MOTOR_LEFT:
    if(currentSpeed < 0) 
      gpio_write(pi, GPIO_DIR1, 1); // set DIRECTION
    else
      gpio_write(pi, GPIO_DIR1, 0);
//    hardware_PWM(pi, GPIO_STEP1, abs((int16_t)(currentSpeed*1000.0)), 500000);
    hardware_PWM(pi, GPIO_STEP1, abs((int16_t)(currentSpeed*2000.0)), 500000);
    break;
  case DIFFDRIVE_MOTOR_RIGHT:
    if(currentSpeed < 0) 
      gpio_write(pi, GPIO_DIR0, 0); // set DIRECTION
    else
      gpio_write(pi, GPIO_DIR0, 1);
//    hardware_PWM(pi, GPIO_STEP0, abs((int16_t)(currentSpeed*1000.0)), 500000);
    hardware_PWM(pi, GPIO_STEP0, abs((int16_t)(currentSpeed*2000.0)), 500000);
    break;
  }

  motors[motor_num].currentSpeed = currentSpeed;

  // set/reset the SLEEP of the motor control
#if 0
  if(abs(motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed) < MOTOR_SPEED_SLEW_SIZE && 
     abs(motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed) < MOTOR_SPEED_SLEW_SIZE)
#else
  if(motors[DIFFDRIVE_MOTOR_LEFT].currentSpeed == 0 && 
     motors[DIFFDRIVE_MOTOR_RIGHT].currentSpeed == 0)
#endif
    gpio_write(pi, GPIO_SLEEP, 0); // set SLEEP (active low)
  else
    gpio_write(pi, GPIO_SLEEP, 1);

#if 0
  ROS_WARN("Motor: %d, Current speed: %f, Target speed: %f", 
           motor_num, currentSpeed, targetSpeed);
#endif
}

// JJ - provide odom transform
#if 1
/*----------------------------------------------------------
 * setEnableOdomTransform() - enable the odom transform
 *--------------------------------------------------------*/
void MiniTurtyBase::setEnableOdomTransform(const bool enabled)
{
  odomTransformEnabled = enabled;
}
#endif

/*----------------------------------------------------------
 * publishOdom() - publish odom data 
 *--------------------------------------------------------*/
void MiniTurtyBase::publishOdom(const double vx, const double vth)
{
  double vy = 0;
  static double x = 0;
  static double y = 0;
  static double th = 0;
  ros::Time current_time = ros::Time::now();
  static ros::Time last_time = current_time;
  static ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);

#if 1
//  ROS_WARN("odom vx: %f, vy: %f, vth: %f\r", vx, vy, vth);
  printf("odom vx: %f, vy: %f, vth: %f\n", vx, vy, vth);
#endif

  // compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  // integrate to track position
  x += delta_x;
  y += delta_y;
  th += delta_th;

  // since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  // send the odom-->base_footprint transform: only if NOT using robot_pose_ekf!
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
//    odom_trans.header.frame_id = "odom_combined";
  odom_trans.header.frame_id = "odom";
//    odom_trans.child_frame_id = "base_link";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);
#if 0
  ROS_INFO("odom x: %f, y: %f, z-rot: %f", x, y, th);
#endif

  // publish the odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
//  odom.header.frame_id = "odom_combined";
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // set the position and orientation
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // TODO: these were repurposed from nxt_ros:: base_odometry.py, check the validity
  odom.pose.covariance[0] = 0.00001;    // x - are all these correct?
  odom.pose.covariance[7] = 0.00001;    // y
  odom.pose.covariance[14] = 10;    // z (set high because robot does not move up/down?)
  odom.pose.covariance[21] = 1;   // rot x
  odom.pose.covariance[28] = 1;   // rot y
  odom.pose.covariance[35] = 1;   // rot z

  // set the velocity
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  last_time = current_time;

  // publish the odom message
  odom_pub.publish(odom);
}

/*----------------------------------------------------------
 * setCmdVelTopic() - set the cmd_vel topic name 
 *--------------------------------------------------------*/
void MiniTurtyBase::setCmdVelTopic(const std::string topic_name)
{
  cmd_vel_sub = node.subscribe(topic_name, 100, &MiniTurtyBase::cmdVelCallback, this);
}

/*----------------------------------------------------------
 * cmdVelCallback()
 *--------------------------------------------------------*/
void MiniTurtyBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
//  motors[DIFFDRIVE_MOTOR_LEFT].targetSpeed = twist->linear.x-twist->angular.z;
//  motors[DIFFDRIVE_MOTOR_RIGHT].targetSpeed = twist->linear.x+twist->angular.z;
//  motors[DIFFDRIVE_MOTOR_LEFT].targetSpeed = twist->linear.x-(twist->angular.z/8.25);
//  motors[DIFFDRIVE_MOTOR_RIGHT].targetSpeed = twist->linear.x+(twist->angular.z/8.25);
  motors[DIFFDRIVE_MOTOR_LEFT].targetSpeed = twist->linear.x-(twist->angular.z/16);
  motors[DIFFDRIVE_MOTOR_RIGHT].targetSpeed = twist->linear.x+(twist->angular.z/16);

#if 1
  ROS_WARN("twist linear.x: %f, y: %f, angular.z: %f\r", 
            twist->linear.x, twist->linear.y, twist->angular.z);  
#endif
}

void MiniTurtyBase::setMotorSpeed(DIFFDRIVE_MOTOR motor_num, double speed)
{
  motors[motor_num].targetSpeed = speed;
}

/////////////////////////////////////////////////////////////////////////

#if !MINI_USE_CONSOLE
void sig_handler(int s)
{
  if (s == 2) {
    got_ctrl_c = true; 
  }
}
#endif

int init_gpio(void)
{
  int pi = pigpio_start(NULL, NULL);
  if (pi < 0) goto error;

  set_mode(pi, GPIO_SLEEP, PI_OUTPUT);
  set_mode(pi, GPIO_DIR0, PI_OUTPUT);
  set_mode(pi, GPIO_DIR1, PI_OUTPUT);
  set_mode(pi, GPIO_STEP0, PI_OUTPUT);
  set_mode(pi, GPIO_STEP1, PI_OUTPUT);

error:
  return pi;
}

/*---------------------------------------------------------------
 * console functions 
 *-------------------------------------------------------------*/

#if MINI_USE_CONSOLE
struct termios orig_termios;

void consoleResetTerminalMode();
int consoleKbHit();
int consoleGetCh();

static bool started = true;
static double left_motor_speed = 0;
static double right_motor_speed = 0;

char processConsole(MiniTurtyBase *mini)
{
  char kbd_ch = 0;
  int i;
// JJ - add theta trim command
#if 0
  static int thetaTrim = 0;
#endif

  if (consoleKbHit())
  {
    kbd_ch = consoleGetCh();
    
    switch (kbd_ch)
    {
    case 'b':
    case 0x42:
      left_motor_speed -= 0.1;
      right_motor_speed -= 0.1;
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_LEFT, left_motor_speed);
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_RIGHT, right_motor_speed);
      printf("Got BWD\n");
      break;
    case 'f':
    case 0x41:
      left_motor_speed += 0.1;
      right_motor_speed += 0.1;
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_LEFT, left_motor_speed);
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_RIGHT, right_motor_speed);
      printf("Got FWD\n");
      break;
    case 'l':
    case 0x44:
#if 1
      left_motor_speed -= 0.025;
      right_motor_speed += 0.025;
#else
      left_motor_speed -= 0.05;
      right_motor_speed += 0.05;
#endif
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_LEFT, left_motor_speed);
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_RIGHT, right_motor_speed);
      printf("Got LEFT\n");
      break;
    case 'r':
    case 0x43:
#if 1
      left_motor_speed += 0.025;
      right_motor_speed -= 0.025;
#else
      left_motor_speed += 0.05;
      right_motor_speed -= 0.05;
#endif
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_LEFT, left_motor_speed);
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_RIGHT, right_motor_speed);
      printf("Got RIGHT\n");
      break;
    case 's':
      left_motor_speed = 0.0;
      right_motor_speed = 0.0;
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_LEFT, left_motor_speed);
      mini->setMotorSpeed(DIFFDRIVE_MOTOR_RIGHT, right_motor_speed);
      printf("Got STOP\n");
      break;

      // JJ - add theta trim command
#if 0
    case '+':
      if (pScanner->theta_trim_ < 128) {
        pScanner->theta_trim_ += 1;
      }
      pScanner->dxlSetThetaTrim(pScanner->theta_trim_);
      break;
    case '-':
      if (pScanner->theta_trim_ > -128) {
        pScanner->theta_trim_ -= 1;
      }
      pScanner->dxlSetThetaTrim(pScanner->theta_trim_);
      break;
#endif
    default:
      break;
    }
  }

  return kbd_ch;
}

void consoleResetTerminalMode()
{
  tcsetattr(0, TCSANOW, &orig_termios);
}

void consoleSetTerminalMode()
{
  struct termios new_termios;

  /* take two copies - one for now, one for later */
  tcgetattr(0, &orig_termios);
  memcpy(&new_termios, &orig_termios, sizeof(new_termios));

  /* register cleanup handler, and set the new terminal mode */
  atexit(consoleResetTerminalMode);

  cfmakeraw(&new_termios);

  new_termios.c_oflag |= OPOST;

  tcsetattr(0, TCSANOW, &new_termios);
}

int consoleKbHit()
{
  struct timeval tv = { 0L, 0L};

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}

int consoleGetCh()
{
  int r;
  unsigned char c;
  if ((r = read(0, &c, sizeof(c))) < 0)
  {
    return r;
  }
  else
  {
    return c;
  }
}
#endif
