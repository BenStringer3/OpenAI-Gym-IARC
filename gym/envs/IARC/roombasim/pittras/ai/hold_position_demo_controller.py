'''
hold_position_demo_controller.py
'''

from gym.envs.IARC.roombasim.ai import Controller

def hold_position_task_completion_callback(status, message):
    '''
    Callback for Hold Position task completion.
    '''
    print("Hold Position task completed with", status)

class HoldPositionDemoController(Controller):
    '''
    A demo controller tests HoldPositionTask.
    '''

    def setup(self):
        self.holding = False

        # set the initial target
        print('Initiating takeoff...')
        self.task_controller.switch_task(
            'TakeoffTask',
            callback = (lambda a,b: self.move())
        )

    def move(self):
        print('Start movement')
        self.task_controller.switch_task(
            'XYZTranslationTask',
            target = [10,10,2],
            callback = (lambda a,b: self.move())
        )

    def update(self, delta, elapsed, environment):
        if elapsed > 7000 and self.holding == False:
            print('Hold position')
            self.holding = True
            self.task_controller.switch_task(
                'HoldPositionTask',
                hold_duration=5,
                callback=hold_position_task_completion_callback
            )
