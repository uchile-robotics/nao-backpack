/**
 * @file main_node.cpp
 * Implementation of a node that publishes NAO data from UDP communication
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */


#include "BHInterface.h"

// Main
int main(int argc, char** argv)
{
    // node initialization
    ros::init(argc, argv, "bhuman2ros");

    bhuman2ros::BHInterface bh;
    ros::spin();

    ros::shutdown();
    return EXIT_SUCCESS;
}
