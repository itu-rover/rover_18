#!/usr/bin/env python


### THIS IS THE STATE MACHINE FOR ROVER


import rospy
import smach
import smach_ros
from std_msgs.msg import String

chooseVariable = ""
autonomousVariable=""

#####################################################################################################################################
#processing the data that coming form choose_duty

def call_back_choose(data):
    global chooseVariable
    chooseVariable = data.data

#choose_duty state: The purpose is orient the smach depends on choosen duty.
class CHOOSE_DUTY(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['TO_AUTONOMOUS','TO_TERRAIN_TRAVERSAL','TO_EQUIPMENT_SERVICING','TO_SCIENCE_CACHE','KILL'])
        self.receivedCheck = False
        print('On Choose Duty State')

    def execute(self,userdata):
        rospy.Subscriber('choose_duty',String,call_back_choose)
        while self.receivedCheck is False:
            if chooseVariable is '1':
                return 'TO_AUTONOMOUS'
            elif chooseVariable is '2':
                return 'TO_TERRAIN_TRAVERSAL'
            elif chooseVariable is '3':
                return 'TO_EQUIPMENT_SERVICING'
            elif chooseVariable is '4':
                return 'TO_SCIENCE_CACHE'
            elif chooseVariable is '0':
                return 'KILL'
        self.receivedCheck = True

#############################################################################################################################################
#Checking if all compilments that will use in autonomous mission are ready or not.

class CHECK_AUTONOMOUS(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCESS','FAIL','REPEAT'])
    def execute(self,userdata):
        print('On Check Autonomus State')
        return 'SUCCESS'

def call_back_autonomous(data):
    
    global autonomousVariable
    autonomousVariable = data.data
    
#############################################################################################################################################
#The purpose is start the autonomous state via autonomous topic.Also check it.

class MISSION_A(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['REPEAT','FAIL'])
        self.autonomous_pub = rospy.Publisher('autonomous',String,queue_size=100)
        
    def execute(self,userdata):
        autonomous_string = raw_input("Choose 1 for Start, 0 For Exit \n")
        
        
        while autonomous_string is not '0':
            self.autonomous_pub.publish(autonomous_string)
            #while autonomousVariable is not "":
            print (autonomousVariable)          
            
            if autonomousVariable is '0':
                return 'FAIL'
        
            elif autonomousVariable is '1':
                print("It's WORKING")
                return 'FAIL'                  
            
##########################################################################################################################################              

    
class CHECK_TERRAIN_TRAVERSAL(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCESS','REPEAT','FAIL'])
    def execute(self,userdata):
        rospy.loginfo('On prototype control mode')
        return 'FAIL'
class MISSION_TT(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['REPEAT','FAIL'])
    def execute(self,userdata):
        rospy.loginfo('On prototype control mode')
        return 'FAIL'
class CHECK_SCIENCE_CACHE(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCESS','REPEAT','FAIL'])
    def execute(self,userdata):
        rospy.loginfo('On prototype control mode')
        return 'FAIL'
class MISSION_SC(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['REPEAT','FAIL'])
    def execute(self,userdata):
        rospy.loginfo('On prototype control mode')
        return 'FAIL'
class CHECK_EQUIPMENT_SERVICING(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCESS','REPEAT','FAIL'])
    def execute(self,userdata):
        rospy.loginfo('On prototype control mode')
        return 'FAIL'
class MISSION_ES(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['REPEAT','FAIL'])
    def execute(self,userdata):
        rospy.loginfo('On prototype control mode')
        return 'FAIL'
    
    
################################################################################################################################################
def CreateStateMachine():
    #The main state machine is containing 4 sub state machines named : autonomous,terrain traversal, equipment servicing,science cache.
    #create top state machine
    sm_top = smach.StateMachine(outcomes = ['DEAD'])        

    with sm_top:
        #Create sub state machines
        sm_sub_autonomous = smach.StateMachine(outcomes = ['KILL_SUB_A'])                 
        sm_sub_terrain_traversal = smach.StateMachine(outcomes = ['KILL_SUB_TT'])
        sm_sub_equipment_servicing = smach.StateMachine(outcomes = ['KILL_SUB_ES'])
        sm_sub_science_cache = smach.StateMachine(outcomes = ['KILL_SUB_SC'])

        smach.StateMachine.add('CHOOSE_DUTY',CHOOSE_DUTY(),
                                transitions={'TO_AUTONOMOUS':'AUTONOMOUS','TO_TERRAIN_TRAVERSAL':'TERRAIN_TRAVERSAL',
                                             'TO_EQUIPMENT_SERVICING':'EQUIPMENT_SERVICING','TO_SCIENCE_CACHE':'SCIENCE_CACHE','KILL':'DEAD'})

        smach.StateMachine.add('AUTONOMOUS', sm_sub_autonomous,
                               transitions={'KILL_SUB_A':'DEAD'})
        smach.StateMachine.add('TERRAIN_TRAVERSAL', sm_sub_terrain_traversal,
                               transitions={'KILL_SUB_TT':'DEAD'})  
        smach.StateMachine.add('EQUIPMENT_SERVICING', sm_sub_equipment_servicing,
                               transitions={'KILL_SUB_ES':'DEAD'})
        smach.StateMachine.add('SCIENCE_CACHE', sm_sub_science_cache,
                               transitions={'KILL_SUB_SC':'DEAD'})  

        with sm_sub_autonomous:                     
            smach.StateMachine.add('CHECK_AUTONOMOUS',CHECK_AUTONOMOUS(),
                                    transitions={'SUCCESS':'MISSION_A','FAIL':'KILL_SUB_A','REPEAT':'CHECK_AUTONOMOUS'}) 
            smach.StateMachine.add('MISSION_A',MISSION_A(),
                                    transitions={'REPEAT':'MISSION_A','FAIL':'KILL_SUB_A'}) 
        
        with sm_sub_terrain_traversal:
            smach.StateMachine.add('CHECK_TERRAIN_TRAVERSAL',CHECK_TERRAIN_TRAVERSAL(),
                                    transitions={'SUCCESS':'MISSION_TT','FAIL':'KILL_SUB_TT','REPEAT':'CHECK_TERRAIN_TRAVERSAL'})
            smach.StateMachine.add('MISSION_TT',MISSION_TT(),
                                    transitions={'REPEAT':'MISSION_TT','FAIL':'KILL_SUB_TT'})

        with sm_sub_science_cache:
            smach.StateMachine.add('CHECK_SCIENCE_CACHE',CHECK_SCIENCE_CACHE(),
                                    transitions={'SUCCESS':'MISSION_SC','FAIL':'KILL_SUB_SC','REPEAT':'CHECK_SCIENCE_CACHE'})
            smach.StateMachine.add('MISSION_SC',MISSION_SC(),
                                    transitions={'REPEAT':'MISSION_SC','FAIL':'KILL_SUB_SC'})
        with sm_sub_equipment_servicing:
            smach.StateMachine.add('CHECK_EQUIPMENT_SERVICING',CHECK_EQUIPMENT_SERVICING(),
                                    transitions={'SUCCESS':'MISSION_ES','FAIL':'KILL_SUB_ES','REPEAT':'CHECK_EQUIPMENT_SERVICING'})
            smach.StateMachine.add('MISSION_ES',MISSION_ES(),
                                    transitions={'REPEAT':'MISSION_ES','FAIL':'KILL_SUB_ES'})
        
        

        

        
        

        sis = smach_ros.IntrospectionServer('rover_state_machine', sm_top, '/ROVER_SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()

def main():
    rospy.init_node('state_machine')
    
    rospy.Subscriber('autonomous',String,call_back_autonomous)
    rate = rospy.Rate(100)
        
    
    
    while not rospy.is_shutdown():
        CreateStateMachine()
        rospy.loginfo("Created State Machine..")

if __name__ == '__main__':
    main()