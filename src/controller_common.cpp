// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "pose_transform.h"
#include "tf_publisher.h"
#include "scalar_dummy.h"

ORO_LIST_COMPONENT_TYPE(PoseTransform)

ORO_LIST_COMPONENT_TYPE(TfPublisher)

ORO_LIST_COMPONENT_TYPE(ScalarDummy)

ORO_CREATE_COMPONENT_LIBRARY()

