#include <math.h>
#include <std_srvs/Trigger.h>
#include "TaskTrigger.h"
#include "floor_nav/TaskTriggerConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

boost::mutex TaskTrigger::clients_mutex;
TaskTrigger::ClientMap TaskTrigger::clients;

TaskIndicator TaskTrigger::initialize()
{
    boost::unique_lock<boost::mutex> lock(clients_mutex);
    ClientMap::const_iterator it = clients.find(cfg.service);
    if (it == clients.end()) {
        try {
            clients[cfg.service] = env->getNodeHandle().serviceClient<std_srvs::Trigger>(cfg.service);
        } catch (...) {
            ROS_ERROR("Service client creation failed: '%s'",cfg.service.c_str());
            return TaskStatus::TASK_INITIALISATION_FAILED;
        }
    }
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskTrigger::iterate()
{
    boost::unique_lock<boost::mutex> lock(clients_mutex);
    std_srvs::Trigger srv;
    clients[cfg.service].call(srv);
    if (srv.response.success) {
        return TaskStatus::TASK_COMPLETED;
    } else {
        ROS_ERROR("Task trigger failed: %s",srv.response.message.c_str());
        return TaskStatus::TASK_FAILED;
    }
}

TaskIndicator TaskTrigger::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryTrigger);
