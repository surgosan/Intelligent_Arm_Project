#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Servo.h>

// Create a NodeHandle for ROS communication
ros::NodeHandle nh;

// Create Servo objects for each joint and the gripper
Servo joint1;
Servo joint2;
Servo joint3;
Servo joint4;
Servo gripper;

// Callback function for incoming joint command messages
void commandCallback(const std_msgs::Float64MultiArray &msg) {
  // We expect msg.data to contain 5 values: [joint1, joint2, joint3, joint4, gripper]
  if (msg.data_length < 5) {
    return; // Not enough data; ignore this message.
  }

  // Convert joint angles from radians to degrees.
  float j1 = msg.data[0] * 180.0 / PI;
  float j2 = msg.data[1] * 180.0 / PI;
  float j3 = msg.data[2] * 180.0 / PI;
  float j4 = msg.data[3] * 180.0 / PI;
  float g  = msg.data[4] * 180.0 / PI; // Gripper position

  // Constrain values to servo limits (assuming 0 to 180 degrees)
  j1 = constrain(j1, 0, 180);
  j2 = constrain(j2, 0, 180);
  j3 = constrain(j3, 0, 180);
  j4 = constrain(j4, 0, 180);
  g  = constrain(g, 0, 180);

  // Write the converted angles to each servo.
  joint1.write(j1);
  joint2.write(j2);
  joint3.write(j3);
  joint4.write(j4);
  gripper.write(g);
}

// Create a ROS subscriber on the "joint_commands" topic
ros::Subscriber<std_msgs::Float64MultiArray> command_sub("joint_commands", commandCallback);

void setup() {
  // Attach each servo to a specific pin on the OpenCR board.
  // Change these pin numbers to match your wiring.
  joint1.attach(9);   // Base joint
  joint2.attach(10);  // Shoulder joint
  joint3.attach(11);  // Elbow joint
  joint4.attach(12);  // Wrist joint
  gripper.attach(13); // Gripper

  // Initialize ROS node and subscribe to the joint command topic.
  nh.initNode();
  nh.subscribe(command_sub);
}

void loop() {
  // Let ROS process incoming messages.
  nh.spinOnce();
  delay(10); // Small delay to ease processor load.
}
