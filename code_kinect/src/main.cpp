/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Simple Skeleton Sample                               *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "NiTE.h"

#include "NiteSampleUtilities.h"

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

int main(int argc, char** argv)
{
	nite::UserTracker userTracker;
	nite::Status niteRc;

	nite::NiTE::initialize();

	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

	nite::UserTrackerFrameRef userTrackerFrame;
	while (!wasKeyboardHit())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc != nite::STATUS_OK)
		{
			printf("Get next frame failed\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user,userTrackerFrame.getTimestamp());
			if (user.isNew())
			{
				userTracker.startSkeletonTracking(user.getId());
			}
			else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
			{
				const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
                if (head.getOrientationConfidence() > .5)
                printf("User n° %d. Head Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z,
                       head.getOrientation().x, head.getOrientation().y, head.getOrientation().z, head.getOrientation().w);

                const nite::SkeletonJoint& neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
                if (neck.getOrientationConfidence() > .5)
                printf("User n° %d. Neck Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), neck.getPosition().x, neck.getPosition().y, neck.getPosition().z,
                       neck.getOrientation().x, neck.getOrientation().y, neck.getOrientation().z, neck.getOrientation().w);

                const nite::SkeletonJoint& left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
                if (left_shoulder.getOrientationConfidence() > .5)
                printf("User n° %d. Left shoulder Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), left_shoulder.getPosition().x, left_shoulder.getPosition().y, left_shoulder.getPosition().z,
                       left_shoulder.getOrientation().x, left_shoulder.getOrientation().y, left_shoulder.getOrientation().z, left_shoulder.getOrientation().w);

                const nite::SkeletonJoint& right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
                if (right_shoulder.getOrientationConfidence() > .5)
                printf("User n° %d. Right shoulder Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), right_shoulder.getPosition().x, right_shoulder.getPosition().y, right_shoulder.getPosition().z,
                       right_shoulder.getOrientation().x, right_shoulder.getOrientation().y, right_shoulder.getOrientation().z, right_shoulder.getOrientation().w);

                const nite::SkeletonJoint& left_elbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
                if (left_elbow.getOrientationConfidence() > .5)
                printf("User n° %d. Left elbow Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), left_elbow.getPosition().x, left_elbow.getPosition().y, left_elbow.getPosition().z,
                       left_elbow.getOrientation().x, left_elbow.getOrientation().y, left_elbow.getOrientation().z, left_elbow.getOrientation().w);

                const nite::SkeletonJoint& right_elbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
                if (right_elbow.getOrientationConfidence() > .5)
                printf("User n° %d. Right elbow Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), right_elbow.getPosition().x, right_elbow.getPosition().y, right_elbow.getPosition().z,
                       right_elbow.getOrientation().x, right_elbow.getOrientation().y, right_elbow.getOrientation().z, right_elbow.getOrientation().w);

                const nite::SkeletonJoint& left_hand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
                if (left_hand.getOrientationConfidence() > .5)
                printf("User n° %d. Left hand Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), left_hand.getPosition().x, left_hand.getPosition().y, left_hand.getPosition().z,
                       left_hand.getOrientation().x, left_hand.getOrientation().y, left_hand.getOrientation().z, left_hand.getOrientation().w);

                const nite::SkeletonJoint& right_hand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
                if (right_hand.getOrientationConfidence() > .5)
                printf("User n° %d. Right hand Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), right_hand.getPosition().x, right_hand.getPosition().y, right_hand.getPosition().z,
                       right_hand.getOrientation().x, right_hand.getOrientation().y, right_hand.getOrientation().z, right_hand.getOrientation().w);

                const nite::SkeletonJoint& torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
                if (torso.getOrientationConfidence() > .5)
                printf("User n° %d. Torso Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), torso.getPosition().x, torso.getPosition().y, torso.getPosition().z,
                       torso.getOrientation().x, torso.getOrientation().y, torso.getOrientation().z, torso.getOrientation().w);

                const nite::SkeletonJoint& left_hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
                if (left_hip.getOrientationConfidence() > .5)
                printf("User n° %d. Left hip Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), left_hip.getPosition().x, left_hip.getPosition().y, left_hip.getPosition().z,
                       left_hip.getOrientation().x, left_hip.getOrientation().y, left_hip.getOrientation().z, left_hip.getOrientation().w);

                const nite::SkeletonJoint& right_hip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
                if (right_hip.getOrientationConfidence() > .5)
                printf("User n° %d. Right hip Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), right_hip.getPosition().x, right_hip.getPosition().y, right_hip.getPosition().z,
                       right_hip.getOrientation().x, right_hip.getOrientation().y, right_hip.getOrientation().z, right_hip.getOrientation().w);

                const nite::SkeletonJoint& left_knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
                if (left_knee.getOrientationConfidence() > .5)
                printf("User n° %d. Left_knee Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), left_knee.getPosition().x, left_knee.getPosition().y, left_knee.getPosition().z,
                       left_knee.getOrientation().x, left_knee.getOrientation().y, left_knee.getOrientation().z, left_knee.getOrientation().w);

                const nite::SkeletonJoint& right_knee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
                if (right_knee.getOrientationConfidence() > .5)
                printf("User n° %d. Right knee Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), right_knee.getPosition().x, right_knee.getPosition().y, right_knee.getPosition().z,
                       right_knee.getOrientation().x, right_knee.getOrientation().y, right_knee.getOrientation().z, right_knee.getOrientation().w);

                const nite::SkeletonJoint& left_foot = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
                if (left_foot.getOrientationConfidence() > .5)
                printf("User n° %d. Left foot Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), left_foot.getPosition().x, left_foot.getPosition().y, left_foot.getPosition().z,
                       left_foot.getOrientation().x, left_foot.getOrientation().y, left_foot.getOrientation().z, left_foot.getOrientation().w);

                const nite::SkeletonJoint& right_foot = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
                if (right_foot.getOrientationConfidence() > .5)
                printf("User n° %d. Right foot Position(%5.2f, %5.2f, %5.2f) Orientation(%5.2f, %5.2f, %5.2f, %5.2f)\n",
                       user.getId(), right_foot.getPosition().x, right_foot.getPosition().y, right_foot.getPosition().z,
                       right_foot.getOrientation().x, right_foot.getOrientation().y, right_foot.getOrientation().z, right_foot.getOrientation().w);
			}
		}

	}

	nite::NiTE::shutdown();

}
