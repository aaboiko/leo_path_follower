#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h> // for STDIN_FILENO
#include <termios.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

class TeleopLeo
{
public:
    TeleopLeo();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
};

TeleopLeo::TeleopLeo()
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void TeleopLeo::keyLoop()
{
    char c;
    bool dirty = false;
    double linear = 0, angular = 0;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("Use WASD keys to move, and QE keys to rotate.");
    puts("Press 'q' to quit.");

    for (;;)
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        linear = angular = 0;
        ROS_DEBUG("value: 0x%02X\n", c);
        switch (c)
        {
        case KEYCODE_W:
            linear = 1;
            dirty = true;
            ROS_INFO("Up");
            break;
        case KEYCODE_S:
            linear = -1;
            dirty = true;
            ROS_INFO("Down");
            break;
        case KEYCODE_A:
            angular = 1;
            dirty = true;
            ROS_INFO("Left");
            break;
        case KEYCODE_D:
            angular = -1;
            dirty = true;
            ROS_INFO("Right");
            break;
        case KEYCODE_Q:
            angular = 0.5;
            dirty = true;
            ROS_INFO("Rotate Left");
            break;
        case KEYCODE_E:
            angular = -0.5;
            dirty = true;
            ROS_INFO("Rotate Right");
            break;
        }
        geometry_msgs::Twist twist;
        twist.angular.z = angular;
        twist.linear.x = linear;
        if (dirty == true)
        {
            vel_pub_.publish(twist);
            dirty = false;
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_key");
    TeleopLeo teleop_leo;

    if (tcgetattr(kfd, &cooked) < 0)
    {
        ROS_ERROR("Could not get terminal attributes");
        return -1;
    }
    teleop_leo.keyLoop();
    return 0;
}