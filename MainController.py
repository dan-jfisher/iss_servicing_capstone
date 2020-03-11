import HandrailLocator
import MapManager
import CameraController

# This code manages the state of the robot and calls the necessary functions

class MainController:

    def __init__(self):
        self.handrail_locator = HandrailLocator.calibrate_from_package_return_handrail_filter()
        self.map_manager = MapManager.create_sample_map_manager()
        self.camera = CameraController()

    def find_and_id_handrails(self):
        handrail_vectors = self.handrail_locator.get_handrail_vectors()
        id_confidence_list = [self.map_manager.assign_id_and_conf_to_handrail_detection(distance, offset) for
                              distance, offset in handrail_vectors]

    # Initializing robot
        # Instantiate robot object
        # Give it initial state
        # Give it an initial location
        # Give it positions of handrail

    # State driven behavior
        # Do while(all handrails are not cleaned)
            # If state = 1 (searching for handrails)
                # While handrail is not found (state = 1):
                    # HardwareController.find_handrail
                    # Take pictures every so often
                    # Run handrail detection (HandrailLocator.get_handrail_vectors())
                    # If (handrail detected)
                        # MapManager.assignID
                        # Go to state 2

             # If state = 2
                # While (state != 5)
                    # Move to handrail
                    # Update position
                    # HardwareController.go_to_handrail(current postion, handrail position)
                    # If (obstacle)
                        # State = 3
                    # If (handrail)
                        # State = 5

            #If state = 3
                # While (state = 3)
                    # Map.AddObstacle(current position, obstacle position)
                    # Map.PlanNewPath
                        # If (avoided obstacle)
                            # If all handrails clean
                                # State = 4
                            # If detected handrail isn't clean
                                # State = 2
                            # If undetected handrail isn't cleaned
                                # State = 1
            # If state = 4
                # SLAM.FindNearestWall
                # HardwareController.MoveTowardWall(current position, wall position)
                # If squared up with wall
                    #HardwareController.AdjustWallGrip
                # State = 1

            # If state = 5
                # HardwareController.CleanHandrail()
                # State = 1


