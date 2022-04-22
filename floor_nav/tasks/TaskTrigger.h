#ifndef TASK_TRIGGER_H
#define TASK_TRIGGER_H

#include <map>
#include <boost/thread/mutex.hpp>

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskTriggerConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskTrigger : public TaskInstance<TaskTriggerConfig,SimTasksEnv>
    {
        protected:
            typedef std::map<std::string,ros::ServiceClient> ClientMap;
            // Adding a mutex in the unlikely case that two trigger may be called in background tasks...
            static boost::mutex clients_mutex;
            static ClientMap clients;
        public:
            TaskTrigger(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskTrigger() {};

            virtual TaskIndicator initialize();

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryTrigger : public TaskDefinition<TaskTriggerConfig, SimTasksEnv, TaskTrigger>
    {

        public:
            TaskFactoryTrigger(TaskEnvironmentPtr env) : 
                Parent("Trigger","Call a trigger service",false,env) {}
            virtual ~TaskFactoryTrigger() {};
    };
};

#endif // TASK_TRIGGER_H
