INFO_STATE = True
WARNING_STATE = True
ERROR_STATE = False

def INFO(msg):
    global INFO_STATE
    if INFO_STATE:
        print "[ INFO ] " + msg

def WARN(msg):
    global WARNING_STATE
    if WARNING_STATE:
        print "[ WARNING ] " + msg

def ERROR(msg):
    global ERROR_STATE
    if ERROR_STATE:
        print "[ ERROR ] " + msg
