#!/usr/bin/python3

import rclpy, sys, random
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from smarc_mission_msgs.action import GotoWaypoint
from smarc_mission_msgs.msg import Topics as MissionTopics


class ActionClientExample():
    """
    A dead-simple client to call the server in this package.
    This is basically an annotated version of:
    https://github.com/ros2/examples/tree/humble/rclpy/actions/minimal_action_client
    You can find some other examples there that do slightly different things.
    """
    def __init__(self,
                 node: Node) -> None:
        self._node = node

        # the action name here should be in a Topics message
        # like how we have topic names in the other files here.
        # I am skipping that now.
        self._ac = ActionClient(node=self._node,
                                action_type=GotoWaypoint,
                                action_name=MissionTopics.GOTO_WP_ACTION)
        
        # to check if we even have a running server later
        self._goal_handle = None
        
    def _loginfo(self, s):
        self._node.get_logger().info(s)


    def _goal_response_cb(self, future):
        """
        We will register this method to the action client when we
        send a goal later.
        The action client will call this when we get a response from
        the server: It could accept or reject our goal, the future here
        will have that information for us to handle.
        """
        goal_handle = future.result() # wait for it
        if not goal_handle.accepted:
            self._loginfo("Goal rejected. Sadness")
            return
        
        self._loginfo("Goal accepted. Hurray")
        # we wanna interact with this goal later, maybe. 
        self._goal_handle = goal_handle

        # the goal handle has a result in the future
        # we want to be notified of this result in our callback
        # so we add our own callback to it
        # Why assign it to a self variable? To keep the future around in memory.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_cb)


    def _get_result_cb(self, future):
        # the future contains the result that the action server
        # will fill in
        # as well as the status of the goal.
        # check the options under GoalStatus. to see what
        # statuses an action server can be in.
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._log.info(f"Success: {result.reached_waypoint}")
        else:
            self._loginfo(f"NOT Success: Status:{status}, result:{result.reached_waypoint}")

        # we got _a_ result. Kind of up to you to think about
        # what to do next. Send again? Wait? Depends on the point
        # of the client.
        # here, we just quit.
        rclpy.shutdown()


    def _feedback_cb(self, feedback):
        # the feedback object has a field "feedback" 
        # this field contains the object we defined in smarc_mission_msgs/action/GotoWaypoint.action
        self._loginfo(f"Got feedback from server: {feedback.feedback.feedback_message}")


    def send_goal(self):
        # the server might not be alive.
        # wait a bit for it, timeout can be None to wait forever.
        # in some instances, you might need it _now_ or _never_
        # then you can use the timeout and check the return.
        # true if the server is ready.
        self._loginfo("Waiting for server to come alive")
        server_is_ready = self._ac.wait_for_server(timeout_sec=30)

        if not server_is_ready:
            self._loginfo("Server was not availble, quitting!")
            rclpy.shutdown()
            return
        
        goal_msg = GotoWaypoint.Goal()
        # fill in the goal, we just hardcode 1000 because why not
        goal_msg.waypoint.travel_rpm = float(1000)
        
        self._loginfo(f"Sending goal: {goal_msg}")

        # we could've sent the goal non-async as well, but not blocking
        # while waiting for an action to complete is better.
        # Why? Because in _real_ clients, maybe the user of the
        # client will want to cancel, send another goal etc.
        # and blocking would prevent these.
        self._send_goal_future = self._ac.send_goal_async(
            goal=goal_msg,
            feedback_callback=self._feedback_cb)
        
        # we want the future to call our callback when its ready to do so
        self._send_goal_future.add_done_callback(self._goal_response_cb)

        # we sent the goal, now what?
        # well, thats up to you. 
        # if you are a BT, maybe have a tick() method that checks
        # the status of the goal which the _feedback_cb method
        # will write somewhere?

        # here, i'm gonna pre-empt the goal randomly every 2 seconds
        self._cancel_timer = self._node.create_timer(2, self._cancel_goal_maybe)

    def _cancel_goal_maybe(self):
        if(random.randint(0,10) >= 5):
            self._loginfo("Rolled high, not cancelling!")
            return
        
        if(not self._goal_handle):
            self._loginfo("No goal handle, can't cancel what doesn't exist!")
            return
        
        self._loginfo("Rolled low, cancelling goal")
        # again, using futures, in case there are things we want to do
        # while waiting for the server to respond to our cancel request
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_goal_cb)
        # stop that timer that calls this method, it worked, dont repeat.
        self._cancel_timer.cancel()


    def _cancel_goal_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self._loginfo("Goal cancelled")
        else:
            self._loginfo("Cancel failed, PANIC")

        # again, maybe dont shutdown if this client is supposed to be used multiple times
        rclpy.shutdown()
        


        



def main():
    # create a node and our objects in the usual manner.
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("ActionClientNode")
    
    ac = ActionClientExample(node)

    ac.send_goal()
    
    rclpy.spin(node)


if __name__ == "__main__":
    main()