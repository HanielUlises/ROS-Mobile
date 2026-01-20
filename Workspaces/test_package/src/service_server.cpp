#include "rclcpp/rclcpp.hpp"
#include "test_package/srv/odd_even_check.hpp"

typedef test_package::srv::OddEvenCheck OddEvenCheck;

class OddEvenCheckServiceNode : public rclcpp::Node{
    public:
        OddEvenCheckServiceNode() : Node ("odd_even_check_service_node"){
            service_server_ = this -> create_service<OddEvenCheck> (
                "odd_even_check",
                std::bind(&OddEvenCheckServiceNode::check_num_odd_even, this, std::placeholders::_1,
                                                                              std::placeholders::_2)
            );
        }
    private:
        void check_num_odd_even(const OddEvenCheck::Request::SharedPtr request,
            OddEvenCheck::Response::SharedPtr response){
                int remainder = std::abs(request -> number % 2);
                switch (remainder){
                    case 0:
                        response -> decision = "Even";
                        break;
                    case 1:
                        response -> decision = "Odd";
                }
            }
        rclcpp::Service<OddEvenCheck>::SharedPtr service_server_;
};

int main (int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OddEvenCheckServiceNode> ());
    rclcpp::shutdown();

    return 0;
}