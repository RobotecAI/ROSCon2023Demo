local RobotPathFollow = 
{
    Properties =
    {
            RobotEntity = { default = EntityId() },
            PathEntityId= {default = EntityId()},
            Speed = 1.0,
            SteeringGainCrossTrack = 1.0,
            SteeringGainLateral = 1.0,
            LookAhead = 1.0,
            RobotForwardAxis = {default = Vector3.ConstructFromValues(0.0,1.0,0.0) },
            RobotRightAxis = {default = Vector3.ConstructFromValues(1.0,0.0,0.0) },
            Topic = { default = "cmd_vel" },
            StopTopic = { default = "distance_error" },
            Debug = true,
            StartupDelay = {default = 10}
    },
    Spline = nil,
    SplineTransform = nil,
    CurrentTime = 0,
    DepthWarningStatus = 0

}

function RobotPathFollow:OnStdMsgInt32(message)
    self.DepthWarningStatus = message
end

function RobotPathFollow:OnActivate()     
     self.tickBusHandler = TickBus.CreateHandler(self,  0)
     self.tickBusHandler:Connect()
     Debug.Log(" StopTopic " .. self.Properties.StopTopic)
     if #self.Properties.StopTopic > 0 then
        self.ros2BusHandler = SubscriberNotificationsBus.Connect(self, self.Properties.StopTopic)
        SubscriberRequestBus.Broadcast.SubscribeToStdMsgInt32(self.Properties.StopTopic)
     end

end

function RobotPathFollow:OnTick(deltaTime, timePoint)

	assert(self.Properties.RobotEntity ~= EntityId() , "No robot entity set")
	assert(self.Properties.PathEntityId ~= EntityId() , "No path entity set")

	
	if deltaTime > 0.25 then
		return 
	end

	-- startup delay
    self.CurrentTime = self.CurrentTime  + deltaTime
    if self.CurrentTime  < self.Properties.StartupDelay then
        return
    end
	
	if self.Spline == nil then
		self.Spline = SplineComponentRequestBus.Event.GetSpline(self.Properties.PathEntityId)
	end
	
	local linear = 0
	local angular = 0
	
	if self.Spline ~= nil then
		local robotPosition = TransformBus.Event.GetWorldTranslation(self.Properties.RobotEntity)
		assert(robotPosition~=nil, "robotPosition is missing")
		local robotPose = TransformBus.Event.GetWorldTM(self.Properties.RobotEntity) 
		assert(robotPose~=nil, "robotPose is missing")
		local robotTangentAxis = Transform.GetBasisX(robotPose)
		assert(robotTangentAxis~=nil, "robotTangentAxis is missing")
		
				
		robotPoseInv = Transform.Clone(robotPose)
		Transform.Invert(robotPoseInv)
		
		lookAhead = self.Properties.LookAhead
		probePosition = robotPosition + Transform.TransformVector(robotPose, self.Properties.RobotForwardAxis * lookAhead )
		assert(probePosition~=nil, "probePosition is missing")
		
		splineTransform = TransformBus.Event.GetWorldTM(self.Properties.PathEntityId)
	
		splineTransformInv = Transform.Clone(splineTransform)
		Transform.Invert(splineTransformInv)
		
		probePositionInSplineFrame = splineTransformInv * probePosition
		
		local nearestAddress = self.Spline:GetNearestAddressPosition(probePositionInSplineFrame)
		local isClosed = self.Spline:IsClosed()
		local nearestPosition = self.Spline:GetPosition(nearestAddress.splineAddress)

		-- Get tangent for splie
		local tangent = self.Spline:GetTangent(nearestAddress.splineAddress)
		assert(tangent~=nil, "tangent is missing")
		local nearestPositionWorld = splineTransform * nearestPosition	
		local tangentVectorWorld = splineTransform.TransformVector(splineTransform, tangent)
		assert(tangentVectorWorld~=nil, "tangentVectorWorld is missing")
		
		
		
		nearestPositionRobotFrame = robotPoseInv * nearestPositionWorld
		crossTrackError = Vector3.Dot(nearestPositionRobotFrame, self.Properties.RobotRightAxis)

		errorVector = Vector3.Cross(robotTangentAxis, tangentVectorWorld)
		assert(errorVector~=nil, "errorVector is missing")
		
		local headingError = math.asin(Vector3.GetLength(errorVector))
		if Vector3.GetElement(errorVector,2) < 0 then
			headingError = - headingError
		end

		--headingError = Vector3.GetLength(errorVector)


		local steeringAngle =  - self.Properties.SteeringGainCrossTrack * crossTrackError + self.Properties.SteeringGainLateral * headingError
		local speed = self.Properties.Speed
		if self.DepthWarningStatus == 1 then
			speed = speed * 0.5
		end
		if self.DepthWarningStatus == 2 then
			speed = 0
		end
		
		-- check if we are in the end of path 
		
		if math.abs(headingError) > 0.5  then
			steeringAngle =  self.Properties.SteeringGainLateral * headingError
			speed = 0.0
		end
		
		if self.Properties.Debug then 
			
			DebugDrawRequestBus.Broadcast.DrawSphereAtLocation(nearestPositionWorld, 0.2, Color.ConstructFromValues(255,0,0,255), 0)
			local tangentLocation = nearestPositionWorld + tangentVectorWorld 
			DebugDrawRequestBus.Broadcast.DrawSphereAtLocation(tangentLocation, 0.1, Color.ConstructFromValues(0,255,0,255), 0)
			
			
			DebugDrawRequestBus.Broadcast.DrawSphereAtLocation(robotPosition, 0.2, Color.ConstructFromValues(255,0,0,255), 0)
			DebugDrawRequestBus.Broadcast.DrawSphereAtLocation(robotPosition+robotTangentAxis, 0.1, Color.ConstructFromValues(0,255,0,255), 0)
			
			DebugDrawRequestBus.Broadcast.DrawTextAtLocation(nearestPositionWorld + Vector3.CreateAxisZ(2.5), 'FPGA collision ' .. tostring(self.DepthWarningStatus), Color.ConstructFromValues(255,225,0,255), 0)
			
			DebugDrawRequestBus.Broadcast.DrawTextAtLocation(nearestPositionWorld + Vector3.CreateAxisZ(1), 'crosstrack ' .. string.format("%.3f",crossTrackError), Color.ConstructFromValues(255,0,0,255), 0)
			DebugDrawRequestBus.Broadcast.DrawTextAtLocation(nearestPositionWorld + Vector3.CreateAxisZ(2), 'heading '.. string.format("%.3f",headingError), Color.ConstructFromValues(0,255,0,255), 0)
			DebugDrawRequestBus.Broadcast.DrawTextAtLocation(nearestPositionWorld + Vector3.CreateAxisZ(1.5), 'speed '.. string.format("%.1f",speed) .. ' angular '.. string.format("%.3f",steeringAngle), Color.ConstructFromValues(0,255,255,255), 0)
		
		end 
		
		-- PublisherRequestBus.Broadcast.PublishAckermannDriveMsg(self.Properties.Topic, steeringAngle, 0.0, speed, 0.0, 0.0)
			PublisherRequestBus.Broadcast.PublishGeometryMsgsTwist(self.Properties.Topic,
				Vector3.CreateAxisX(speed),
				Vector3.CreateAxisZ(steeringAngle))
	end

end

return RobotPathFollow