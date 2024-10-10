#include <ros/ros.h>
#include <get_gps_from_map/get_gps_from_map_srv.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

bool request_for_GPS_ref_update = false;
bool request_for_GPS = false;
bool exit_loop = false;
bool call_service = false;
bool service_done = false;

sensor_msgs::NavSatFix reference_GPS;

char a;
std::mutex mtx_request_for_GPS_ref_update, mtx_request_for_GPS, mtx_call_service;

void update_bool_variable()
{
   while (true)
   {
      std::cout << "\n\nPress r for GPS reference update \np for get transformed GPS points \ne to exit loop\n";
      std ::cin >> a;

      if (a == 'r')
      {
         std::lock_guard<std::mutex> l1(mtx_request_for_GPS_ref_update);
         request_for_GPS_ref_update = true;

         std::lock_guard<std::mutex> l2(mtx_call_service);
         call_service = true;
      }

      if (a == 'p')
      {
         std::lock_guard<std::mutex> l3(mtx_request_for_GPS);
         request_for_GPS = true;

         std::lock_guard<std::mutex> l4(mtx_call_service);
         call_service = true;
      }

      if (a == 'e')
      {
         exit_loop = true;
         break;
      }

      while (call_service)
      {
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "get_gps_from_map_client");
   ros::service::waitForService("get_gps_from_map_srv_server");
   std::cout << "Server is ready\n";

   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<get_gps_from_map::get_gps_from_map_srv>("get_gps_from_map_srv_server");
   get_gps_from_map::get_gps_from_map_srv srv;
   std::thread th1(update_bool_variable);

   // //// Publisher for publishing the GPS reference data to visualize on RVIZ and corresponding msg defination

   // reference_GPS.latitude = 24.767105598822916;
   // reference_GPS.longitude = 55.36987525111471;

   reference_GPS.latitude = 24.767456719260924;
   reference_GPS.longitude = 55.371002554893494;

   ros::Publisher GPS_pub = n.advertise<sensor_msgs::NavSatFix>("GPS_ref_topic", 1000);
   sensor_msgs::NavSatFix msg;
   msg.header.frame_id = "GPS_ref_frame";
   msg.status.status = 0;
   msg.status.service = 0;
   msg.latitude = reference_GPS.latitude;
   msg.longitude = reference_GPS.longitude;
   msg.altitude = 0.0;
   msg.position_covariance_type = 2;

   // //// Broadcast the GPS_ref_frame with respect to the map for visualizing in RVIZ
   tf::TransformBroadcaster br;
   tf::Transform transform;
   transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
   transform.setRotation(tf::Quaternion(0, 0, 0, 1));

   // //// Visulaize the marker array correponding to the transformed GPS coordinates into xyz wrt reference GPS coordinates.
   ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("GPS_marker", 0);
   visualization_msgs::Marker marker;
   marker.header.frame_id = "GPS_ref_frame";
   marker.header.stamp = ros::Time();
   marker.ns = "GPS_ns";
   marker.id = 0;
   marker.type = visualization_msgs::Marker::LINE_STRIP;
   marker.action = visualization_msgs::Marker::ADD;
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0;
   marker.scale.x = 1;
   marker.scale.y = 1.0;
   marker.scale.z = 1.0;
   marker.color.a = 1.0; // Don't forget to set the alpha!
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;

   ros::Rate loop_rate(10);

   while (ros::ok())
   {
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "GPS_ref_frame"));

      if (call_service)
      {
         if (request_for_GPS_ref_update)
         {
            std::cout << "Calling service to upadte reference GPS\n";

            srv.request.reference_GPS = reference_GPS;
            srv.request.request_to_update_reference_GPS = true;
            srv.request.request_for_GPS = false;
         }

         if (request_for_GPS)
         {
            std::cout << "Calling service to get transformed GPS\n";
            srv.request.request_to_update_reference_GPS = false;
            srv.request.request_for_GPS = true;
         }

         client.call(srv);

         transform.setOrigin(tf::Vector3(srv.response.tile_origin_xyz_wrt_reference_GPS.x, srv.response.tile_origin_xyz_wrt_reference_GPS.y, srv.response.tile_origin_xyz_wrt_reference_GPS.z));

         std::vector<geometry_msgs::Point> received_points;
         received_points = srv.response.Transformed_points;

         std::vector<sensor_msgs::NavSatFix> received_GPS;
         received_GPS = srv.response.GPS_output;

         marker.points.clear();

         geometry_msgs::Point p;
         p.x = 0;
         p.y = 0;
         p.z = 0;
         marker.points.push_back(p);

         for (int i = 0; i < received_points.size(); i++)
         {
            std::cout << "\n"
                      << i << " "
                      << received_points[i].x << " "
                      << received_points[i].y << " "
                      << received_points[i].z << "\n"
                      << received_GPS[i].latitude << " "
                      << received_GPS[i].longitude << "\n";

            marker.points.push_back(received_points[i]);
         }

         std::lock_guard<std::mutex> l5(mtx_request_for_GPS_ref_update);
         request_for_GPS_ref_update = false;

         std::lock_guard<std::mutex> l6(mtx_request_for_GPS);
         request_for_GPS = false;

         std::lock_guard<std::mutex> l7(mtx_call_service);
         call_service = false;
      }

      if (exit_loop)
      {
         break;
      }

      GPS_pub.publish(msg);
      vis_pub.publish(marker);
   }

   th1.join();

   return 0;
}