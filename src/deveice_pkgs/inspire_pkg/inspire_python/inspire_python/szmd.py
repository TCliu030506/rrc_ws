import rclpy
from . import inspire_client
def main(args=None):
    # l1_values, l2_values, l3_values = spiral()
    rclpy.init(args=args)
    motor1 = inspire_client.Client(1)
    motor2 = inspire_client.Client(2)  
    motor3 = inspire_client.Client(3) 
    motor4 = inspire_client.Client(4)
    motor5 = inspire_client.Client(5)  
    motor6 = inspire_client.Client(6) 
    
    while rclpy.ok():
        mode = input("mode ")  # 读取用户输入
        if mode == "4f5d":
            user_input=input() 
            user_input2=input()
            motor4.move_fce(int(user_input))
            motor5.move_vel(int(user_input2), 200)

        if mode == "45d":
            user_input=input()  
            user_input2=input()
            motor4.move_vel(int(user_input), 200)
            motor5.move_vel(int(user_input2), 200)

        if mode == "4d5f":
            user_input=input() 
            user_input2=input() 
            motor4.move_vel(int(user_input), 200)
            motor5.move_fce(int(user_input2))

        elif mode == "1fc":   
            user_input=input()    
            motor1.move_fce(user_input)

        elif mode == "1d":   
            user_input=input()    
            motor1.move_vel(user_input,100)

        elif mode == "6fc":   
            user_input=input()    
            motor6.move_fce(user_input)

        elif mode == "6d":   
            user_input=input()    
            motor6.move_vel(user_input,100)

    rclpy.shutdown()
    
if __name__ == '__main__':
    
    main()