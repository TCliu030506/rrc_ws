#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"
#include "joystick_msg/msg/joystick_parameter.hpp"

#include <stdio.h>  
#include <unistd.h>  
#include <string.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <errno.h>  
#include <linux/input.h>  
#include <linux/joystick.h>  

#define XBOX_TYPE_BUTTON    0x01  
#define XBOX_TYPE_AXIS      0x02  

#define XBOX_BUTTON_A       0x00  
#define XBOX_BUTTON_B       0x01  
#define XBOX_BUTTON_X       0x02  
#define XBOX_BUTTON_Y       0x03  
#define XBOX_BUTTON_LB      0x04  
#define XBOX_BUTTON_RB      0x05  
#define XBOX_BUTTON_BACK    0x06
#define XBOX_BUTTON_START   0x07  
#define XBOX_BUTTON_HOME    0x08  
#define XBOX_BUTTON_LO      0x09    /* 左摇杆按键 */  
#define XBOX_BUTTON_RO      0x0a    /* 右摇杆按键 */  

#define XBOX_BUTTON_ON      0x01  
#define XBOX_BUTTON_OFF     0x00  

#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */  
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */  
#define XBOX_AXIS_LT        0x02 
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */  
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */  
#define XBOX_AXIS_RT        0x05  
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */  
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */  

#define XBOX_AXIS_VAL_UP        -32767  
#define XBOX_AXIS_VAL_DOWN      32767  
#define XBOX_AXIS_VAL_LEFT      -32767  
#define XBOX_AXIS_VAL_RIGHT     32767  

#define XBOX_AXIS_VAL_MIN       -32767  
#define XBOX_AXIS_VAL_MAX       32767  
#define XBOX_AXIS_VAL_MID       0x00  

typedef struct xbox_map  
{  
    int     time;  
    int     a;  
    int     b;  
    int     x;  
    int     y;  
    int     lb;  
    int     rb;  
    int     start;  
    int     back;  
    int     home;  
    int     lo;  
    int     ro;  

    int     lx;  
    int     ly;  
    int     rx;  
    int     ry;  
    int     lt;  
    int     rt;  
    int     xx;  
    int     yy;  

}xbox_map_t;  

int xbox_open(const char *file_name)  
{  
    int xbox_fd;  

    xbox_fd = open(file_name, O_RDONLY);  
    if (xbox_fd < 0)  
    {  
        perror("open");  
        return -1;  
    }  

    return xbox_fd;  
}  

int xbox_map_read(int xbox_fd, xbox_map_t *map)  
{  
    int len, type, number, value;  
    struct js_event js;  

    len = read(xbox_fd, &js, sizeof(struct js_event));  
    if (len < 0)  
    {  
        perror("read");  
        return -1;  
    }  

    type = js.type;  
    number = js.number;  
    value = js.value;  

    map->time = js.time;  

    if (type == JS_EVENT_BUTTON)  
    {  
        switch (number)  
        {  
            case XBOX_BUTTON_A:  
                map->a = value;  
                break;  

            case XBOX_BUTTON_B:  
                map->b = value;  
                break;  

            case XBOX_BUTTON_X:  
                map->x = value;  
                break;  

            case XBOX_BUTTON_Y:  
                map->y = value;  
                break;  

            case XBOX_BUTTON_LB:  
                map->lb = value;  
                break;  

            case XBOX_BUTTON_RB:  
                map->rb = value;  
                break;  

            case XBOX_BUTTON_START:  
                map->start = value;  
                break;  

            case XBOX_BUTTON_BACK:  
                map->back = value;  
                break;  

            case XBOX_BUTTON_HOME:  
                map->home = value;  
                break;  

            case XBOX_BUTTON_LO:  
                map->lo = value;  
                break;  

            case XBOX_BUTTON_RO:  
                map->ro = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else if (type == JS_EVENT_AXIS)  
    {  
        switch(number)  
        {  
            case XBOX_AXIS_LX:  
                map->lx = value;  
                break;  

            case XBOX_AXIS_LY:  
                map->ly = value;  
                break;  

            case XBOX_AXIS_RX:  
                map->rx = value;  
                break;  

            case XBOX_AXIS_RY:  
                map->ry = value;  
                break;  

            case XBOX_AXIS_LT:  
                map->lt = value;  
                break;  

            case XBOX_AXIS_RT:  
                map->rt = value;  
                break;  

            case XBOX_AXIS_XX:  
                map->xx = value;  
                break;  

            case XBOX_AXIS_YY:  
                map->yy = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else  
    {  
        /* Init do nothing */  
    }  

    return len;  
}  

void xbox_close(int xbox_fd)  
{  
    close(xbox_fd);  
    return;  
}  

class JoystickInputPublisher : public rclcpp::Node
{
public:
    JoystickInputPublisher() : Node("joystick_input_publisher")
    {
        publisher_ = this->create_publisher<joystick_msg::msg::JoystickParameter>("joystick_input", 80);

        joystick_thread_ = std::thread(std::bind(&JoystickInputPublisher::joystickInputListener, this));
    }

    ~JoystickInputPublisher(){
    }

private:
    void joystickInputListener()
    {
        int xbox_fd;
        xbox_map_t map;
        int len;

        memset(&map, 0, sizeof(xbox_map_t));  

        xbox_fd = xbox_open("/dev/input/js0");  // js1 is input here
        if(xbox_fd < 0)  
        {  
            RCLCPP_ERROR(get_logger(), "OPEN '/dev/input/js0' WRONG, XBOX_FD < 0");
        }  

        while (rclcpp::ok()) {
            len = xbox_map_read(xbox_fd, &map);  
            if (len < 0)  
            {  
                usleep(10*1000);  
                continue;  
            }  

            printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",  
                    map.time, map.a, map.b, map.x, map.y, map.lb, map.rb, map.start, map.back, map.home, map.lo, map.ro,  
                    map.xx, map.yy, map.lx, map.ly, map.rx, map.ry, map.lt, map.rt);  
            fflush(stdout); 
            
            auto msg = joystick_msg::msg::JoystickParameter();
            msg.a = map.a;
            msg.b = map.b;
            msg.lb = map.lb;
            msg.lt = map.lt;
            msg.lx = map.lx;
            msg.ly = map.ly;
            msg.rb = map.rb;
            msg.rt = map.rt;
            msg.rx = map.rx;
            msg.ry = map.ry;
            msg.x = map.x;
            msg.y = map.y;
            msg.xx = map.xx;
            msg.yy = map.yy;
            publisher_->publish(msg);
        }

        xbox_close(xbox_fd);
    }

    rclcpp::Publisher<joystick_msg::msg::JoystickParameter>::SharedPtr publisher_;
    std::thread joystick_thread_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<JoystickInputPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();
     
    return 0;
}
