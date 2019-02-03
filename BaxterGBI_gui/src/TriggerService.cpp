#include "TriggerService.h"

TriggerService::TriggerService(std::string srv_name)
: srv_name(srv_name){
    client = n.serviceClient<std_srvs::Trigger>(srv_name);
}

void TriggerService::operator()() {
    client.call(request);
}
