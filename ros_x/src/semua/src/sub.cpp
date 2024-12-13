#include "../include/inverse.cpp"
#include "ros/ros.h"
#include "JetsonGPIO.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/UInt16.h" // Untuk pesan jarak
#include "std_msgs/UInt16MultiArray.h"
#include <math.h>
#define BUTTON_PIN 26
bool but = false;
int step = 0;
// Variabel global untuk menyimpan perintah aktif dan data IMU
std::string current_command = "";
float imu_yaw = 0.0, imu_roll = 0.0, imu_pitch = 0.0;
int sensor1_data = 0;
int sensor2_data = 0;

int mulai()
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    uint8_t dxl_error = 0; // Dynamixel error
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return 0;
    }

    // IC();
    for (int i = 0; i < 18; i++)
    {
        // Enable Dynamixel#1 Torque
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_idku[i], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", dxl_idku[i]);
        }
    }

    siap();

    // while (1)
    // {
    //     // maju_robot();
    // }

    // portHandler->closePort();
}
void maju()
{
    ROS_INFO("Robot bergerak maju");
    maju_robot(); // Tambahkan logika untuk perintah maju robot di sini
}

void berhenti_robot()
{
    ROS_INFO("Robot berhenti untuk mempertahankan heading.");
    siap();
    // Tambahkan logika untuk menghentikan semua motor robot
}

// Fungsi untuk memutar robot ke kanan
void kanan_robot1()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    kanan_robot();
    // Tambahkan logika untuk memutar robot ke kanan
}

// Fungsi untuk memutar robot ke kiri
void kiri_robot1()
{
    ROS_INFO("Robot berputar ke kiri untuk mengoreksi heading.");
    kiri_robot();
    // Tambahkan logika untuk memutar robot ke kiri
}
void initGPIO()
{
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(BUTTON_PIN, GPIO::IN);
}
void updateButtonStatus()
{
    if (GPIO::input(BUTTON_PIN) == GPIO::LOW)
    {
        but = true;
    
    }
    else
    {
        but = false;
    }
}
// Callback function to process TOF sensor data
void distanceCallback(const std_msgs::String::ConstPtr& msg)
{
    // Mencetak pesan yang diterima
    std::cout << "Data diterima: " << msg->data << std::endl;
       // Logika pergerakan robot
        if (sensor1_data < 80) {
            kiri_robot();
            ROS_INFO("Robot bergerak ke kiri (sensor 1 < 40)");
        } else if (sensor1_data > 80) {
            maju_robot();
            ROS_INFO("Robot bergerak maju (sensor 2 < 80)");
        } else {
            ROS_INFO("Robot diam (tidak ada kondisi yang terpenuhi)");
        }
}
// Callback untuk menerima data IMU
void imuCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 3)
    {
        imu_yaw = msg->data[0];
        imu_roll = msg->data[1];
        imu_pitch = msg->data[2];
        ROS_INFO("IMU Data - Yaw: %.2f, Roll: %.2f, Pitch: %.2f", imu_yaw, imu_roll, imu_pitch);
        if ((imu_yaw >= 350 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 10))
        {
            berhenti_robot(); // Robot berhenti dalam rentang yang diinginkan
        }
        else if (imu_yaw < 350)
        {
            kanan_robot(); // Yaw di bawah 350, putar kanan
        }
        else if (imu_yaw > 10)
        {
            kiri_robot(); // Yaw di atas 10, putar kiri
        }
    }
    else
    {
        ROS_WARN("Data IMU tidak lengkap");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    initGPIO();
    mulai();


    // Subscribing ke topik chatter
    // ros::Subscriber sub = n.subscribe("chatter", 1000, commandCallback);
    // Subscriber untuk menerima data IMU
    ros::Subscriber sub_imu = n.subscribe("imu/euler", 10, imuCallback);
    ros::Subscriber distance_sub = n.subscribe("sensor_distance", 10, distanceCallback);
    // Membuat subscriber untuk topik "vl53l0x_distance"
    // ros::Subscriber yaw_sub = n.subscribe("imu/yaw", 10, yawCallback);
    // ros::Subscriber roll_sub = n.subscribe("imu/roll", 10, rollCallback);
    // ros::Subscriber pitch_sub = n.subscribe("imu/pitch", 10, pitchCallback);

    // Loop utama untuk menjalankan perintah aktif
    ros::Rate loop_rate(33); // Menentukan frekuensi loop (10 Hz)
    while (ros::ok())
    {
        updateButtonStatus();

        if (but)
        {
            step = 1;
            ROS_INFO("tombol ditekan,robot aktif.");
        }
        // Memproses callback ROS
        ros::spinOnce();

        loop_rate.sleep();
    }
    GPIO::cleanup();
    return 0;
}
