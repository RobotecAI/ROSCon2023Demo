local tractor_control = 
{
    Properties =
    {
            TractorEntityId = { default = EntityId() },
            PathEntityId= {default = EntityId()},
            Speed = 1.0,
            SteeringGain = 10.0,
            LookAhead = 1.0,
            TractorForwardAxis = {default = Vector3.ConstructFromValues(0.0,1.0,0.0) },
            TractorRightAxis = {default = Vector3.ConstructFromValues(1.0,0.0,0.0) },
            Topic = { default = "ackermann_vel" },
            Debug = true,
            StartupDelay = {default = 10}
    },
    Spline = nil,
    SplineTransform = nil,
    CurrentTime = 0

}

function tractor_control:OnActivate()     
     self.tickBusHandler = TickBus.CreateHandler(self,  0)
     self.tickBusHandler:Connect()    
end

function tractor_control:OnTick(deltaTime, timePoint)

	assert(self.Properties.TractorEntityId ~= EntityId() , "No tractor entity set")
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
		local tractorPosition = TransformBus.Event.GetWorldTranslation(self.Properties.TractorEntityId)
		assert(tractorPosition~=nil, "tractorPosition is missing")
		local tractorPose = TransformBus.Event.GetWorldTM(self.Properties.TractorEntityId) 
		assert(tractorPose~=nil, "tractorPose is missing")
				
		tractorPoseInv = Transform.Clone(tractorPose)
		Transform.Invert(tractorPoseInv)
		
		lookAhead = self.Properties.LookAhead
		probePosition = tractorPosition + Transform.TransformVector(tractorPose, self.Properties.TractorForwardAxis * lookAhead )
		assert(probePosition~=nil, "probePosition is missing")
		
		splineTransform = TransformBus.Event.GetWorldTM(self.Properties.PathEntityId)
	
		splineTransformInv = Transform.Clone(splineTransform)
		Transform.Invert(splineTransformInv)
		
		probePositionInSplineFrame = splineTransformInv * probePosition
		
		local nearestAddress = self.Spline:GetNearestAddressPosition(probePositionInSplineFrame)
		local isClosed = self.Spline:IsClosed()
		local nearestPosition = self.Spline:GetPosition(nearestAddress.splineAddress)

		local nearestPositionWorld = splineTransform * nearestPosition	
		nearestPositionTractorFrame = tractorPoseInv * nearestPositionWorld
		crossTrackError = Vector3.Dot(nearestPositionTractorFrame, self.Properties.TractorRightAxis)
		lateralError =  Vector3.Dot(nearestPositionTractorFrame, self.Properties.TractorForwardAxis) - lookAhead

		if self.Properties.Debug then 
			DebugDrawRequestBus.Broadcast.DrawSphereAtLocation(nearestPositionWorld, 0.2, Color.ConstructFromValues(255,0,0,255), 0)
			DebugDrawRequestBus.Broadcast.DrawTextAtLocation(nearestPositionWorld + Vector3.CreateAxisZ(1), 'crosstrack ' .. tostring(crossTrackError), Color.ConstructFromValues(255,0,0,255), 0)
			DebugDrawRequestBus.Broadcast.DrawTextAtLocation(nearestPositionWorld + Vector3.CreateAxisZ(2), 'lateral '.. tostring(lateralError), Color.ConstructFromValues(0,255,0,255), 0)
		end 
		
		local steeringAngle =  - self.Properties.SteeringGain * crossTrackError
		local speed = self.Properties.Speed
		
		-- check if we are in the end of path 
		if lateralError < -0.5 and not isClosed then
			speed = 0
		end
		-- PublisherRequestBus.Broadcast.PublishAckermannDriveMsg(self.Properties.Topic, steeringAngle, 0.0, speed, 0.0, 0.0)
		   PublisherRequestBus.Broadcast.PublishGeometryMsgsTwist(self.Properties.Robot.Topic,
               Vector3.CreateAxisX(self.Properties.Robot.Speed),
               Vector3.CreateAxisZ(steeringAngle))
	end

end

return tractor_control