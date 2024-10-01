#include <ros/ros.h>
#include <get_gps_from_map/get_gps_from_map_srv.h>
#include <thread>
#include <mutex>
#include <chrono>

bool request_for_GPS_ref_update = false;
bool request_for_GPS = false;
bool exit_loop = false;
bool call_service = false;
bool service_done = false;
float ref_latitude = 24.767105598822916;
float ref_longitude = 55.36987525111471;
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

   while (ros::ok())
   {

      if (call_service)
      {
         if (request_for_GPS_ref_update)
         {
            std::cout << "Calling service to upadte reference GPS\n";
            srv.request.ref_lat = ref_latitude;
            srv.request.ref_lon = ref_longitude;
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

         std::vector<float> input_x, input_y, input_z;
         input_x = srv.response.x;
         input_y = srv.response.y;
         input_z = srv.response.z;

         std::cout << "Received Response:\n";
         for (int i = 0; i < input_x.size(); i++)
         {
            std::cout << input_x[i] << " " \
                      << input_y[i] << " " \
                      << input_z[i] << "\n";
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
   }

   th1.join();

   return 0;
}