class AvoidObstacle(Behaviour):
    def __init__(self, name="AvoidObstacle"):
        super(AvoidObstacle, self).__init__(name)
        self.started = False
        self.start_time = None

    def initialise(self):
        self.started = True
        self.start_time = rospy.Time.now()
        print("Avoiding obstacle...")

    def update(self):
        if not self.started:
            return Status.FAILURE
        
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        
        if elapsed < 1.0:
            # Rotate away from obstacle
            publish_twist(linear_x=0.0, angular_z=0.6)
            return Status.RUNNING
        elif elapsed < 2.5:
            # Move forward while turning slightly
            publish_twist(linear_x=0.3, angular_z=0.3)
            return Status.RUNNING
        else:
            publish_twist(0.0, 0.0)
            self.started = False
            print("Obstacle avoided")
            return Status.SUCCESS

    def terminate(self, new_status):
        self.started = False
