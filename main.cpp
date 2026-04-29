#include "bitbot_cifx/kernel/cifx_kernel.hpp"
#include "bitbot_kernel/utils/cpu_affinity.h"
#include "bitbot_kernel/utils/priority.h"

#include "user_func.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/mman.h>

int main(int argc, char const *argv[])
{
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
	{
		printf("mlockall failed: \n");
		if (errno == ENOMEM)
		{
			printf("\nIt is likely your user does not have enough memory limits, you can change the limits by adding the "
						 "following line to /etc/security/limits.conf:\n\n");
		}
		return -1;
	}
	if (!setProcessHighPriority(90))
	{
		printf("Failed to set process scheduling policy\n");
		return -1;
	}

	CifxKernel kernel("../../sc_bhr8fc2.xml");

	kernel.RegisterConfigFunc(ConfigFunc);

	kernel.RegisterEvent("init_pos", static_cast<bitbot::EventId>(Events::InitPos), [](bitbot::EventValue, UserData &)
											 { return static_cast<bitbot::StateId>(States::InitPos); });

	kernel.RegisterEvent("maintain_pos", static_cast<bitbot::EventId>(Events::MaintainPos), [](bitbot::EventValue, UserData &)
											 { return static_cast<bitbot::StateId>(States::MaintainPos); });
											 
	kernel.RegisterEvent("to_fall_pos1", static_cast<bitbot::EventId>(Events::ToFallPos1), [](bitbot::EventValue, UserData &)
											 { return static_cast<bitbot::StateId>(States::ToFallPos1); });

	kernel.RegisterEvent("to_fall_pos2", static_cast<bitbot::EventId>(Events::ToFallPos2), [](bitbot::EventValue, UserData &)
											 { return static_cast<bitbot::StateId>(States::ToFallPos2); });
											 
	kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting), &StateWaiting, {static_cast<bitbot::EventId>(Events::InitPos)});
	
	kernel.RegisterState("init_pos", static_cast<bitbot::StateId>(States::InitPos), &StateInitPos, {static_cast<bitbot::EventId>(Events::MaintainPos)});
	
	kernel.RegisterState("maintain_pos", static_cast<bitbot::StateId>(States::MaintainPos), &StateMaintainPos, {static_cast<bitbot::EventId>(Events::ToFallPos1), static_cast<bitbot::EventId>(Events::ToFallPos2)});

	kernel.RegisterState("to_fall_pos1", static_cast<bitbot::StateId>(States::ToFallPos1), &StateToFallPos1, {});

	kernel.RegisterState("to_fall_pos2", static_cast<bitbot::StateId>(States::ToFallPos2), &StateToFallPos2, {});

	kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));

	kernel.Run();
	return 0;
}
