#!/usr/bin/env python3
"""
Final Project - Autonomous Pick & Place State Machine (FAKE nodes)
01208567 ROS for Mechanical Engineering - In-class assignment (C13)

State design:
  INIT -> NAV_STATION1 -> DETECT_OBJECT -> PICK -> NAV_STATION2
       -> DETECT_STATION -> PLACE -> RETURN_HOME -> [succeeded]
  Each task state: succeeded -> next state, failed -> retry itself
  (max 3 tries), then -> aborted.
Run:  python3 final_project_state_machine.py
"""
import random
import time

import rclpy
import smach

FAIL_RATE = 0.2   # fake failure chance
MAX_RETRY = 3


class FakeTask(smach.State):
    """Fake state: waits a bit and returns succeeded/failed randomly."""

    def __init__(self, node, name, msg, out_keys=None):
        super().__init__(outcomes=['succeeded', 'failed', 'aborted'],
                         input_keys=['retry'], output_keys=['retry'] + (out_keys or []))
        self.node, self.name, self.msg = node, name, msg
        self.out_keys = out_keys or []

    def execute(self, ud):
        log = self.node.get_logger()
        log.info(f'[{self.name}] {self.msg} ...')
        time.sleep(1.0)                      # pretend to work
        if random.random() < FAIL_RATE:
            ud.retry += 1
            log.warn(f'[{self.name}] failed ({ud.retry}/{MAX_RETRY})')
            return 'aborted' if ud.retry >= MAX_RETRY else 'failed'
        for k in self.out_keys:              # fake result, e.g. object pose
            setattr(ud, k, (round(random.uniform(0, 1), 2), round(random.uniform(0, 1), 2)))
        ud.retry = 0
        log.info(f'[{self.name}] done')
        return 'succeeded'


class Init(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['succeeded'], output_keys=['retry', 'home'])
        self.node = node

    def execute(self, ud):
        ud.retry, ud.home = 0, (0.0, 0.0)
        self.node.get_logger().info('[INIT] save home pose, check nav & arm ready')
        return 'succeeded'


def main():
    rclpy.init()
    node = rclpy.create_node('pick_place_sm')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    tasks = [  # (state name, message, output keys, next state)
        ('NAV_STATION1',   'navigate to station 1',            [],             'DETECT_OBJECT'),
        ('DETECT_OBJECT',  'detect object on desk',            ['obj_pose'],   'PICK'),
        ('PICK',           'arm picks object',                 [],             'NAV_STATION2'),
        ('NAV_STATION2',   'navigate to station 2',            [],             'DETECT_STATION'),
        ('DETECT_STATION', 'detect place location',            ['place_pose'], 'PLACE'),
        ('PLACE',          'arm places object',                [],             'RETURN_HOME'),
        ('RETURN_HOME',    'navigate back to initial location', [],            'succeeded'),
    ]
    with sm:
        smach.StateMachine.add('INIT', Init(node), transitions={'succeeded': 'NAV_STATION1'})
        for name, msg, keys, nxt in tasks:
            smach.StateMachine.add(
                name, FakeTask(node, name, msg, keys),
                transitions={'succeeded': nxt, 'failed': name, 'aborted': 'aborted'})

    result = sm.execute()
    node.get_logger().info(f'State machine finished: {result}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
