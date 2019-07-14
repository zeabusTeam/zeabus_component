#!/usr/bin/env python2
import matplotlib.pyplot as plt
import rospy
from zeabus_utility.msg import ControlFuzzy ,FuzzyParam

value_ec_x = [0]
value_ef_x = [0]
value_dc_x = [0]
value_df_x = [0]
value_fc_x = [0]
value_ff_x = [0]
value_oc_x = [0]
value_of_x = [0]  
# e = error ,d = diff ,f = force ,o = output ,c = crisp_data ,f = fuzzy_data
value_ec_y = [0]
value_ef_y = [0]
value_dc_y = [0]
value_df_y = [0]
value_fc_y = [0]
value_ff_y = [0]
value_oc_y = [0]
value_of_y = [0]
value_ec_z = [0]
value_ef_z = [0]
value_dc_z = [0]
value_df_z = [0]
value_fc_z = [0]
value_ff_z = [0]
value_oc_z = [0]
value_of_z = [0]
value_ec_roll = [0]
value_ef_roll = [0]
value_dc_roll = [0]
value_df_roll = [0]
value_fc_roll = [0]
value_ff_roll = [0]
value_oc_roll = [0]
value_of_roll = [0]
value_ec_pitch = [0]
value_ef_pitch = [0]
value_dc_pitch = [0]
value_df_pitch = [0]
value_fc_pitch = [0]
value_ff_pitch = [0]
value_oc_pitch = [0]
value_of_pitch = [0]
value_ec_yaw = [0]
value_ef_yaw = [0]
value_dc_yaw = [0]
value_df_yaw = [0]
value_fc_yaw = [0]
value_ff_yaw = [0]
value_oc_yaw = [0]
value_of_yaw = [0]

error_crisp_x = 0
error_fuzzy_x = 0
diff_crisp_x = 0
diff_fuzzy_x = 0
force_crisp_x = 0
force_fuzzy_x = 0
output_crisp_x = 0
output_fuzzy_x = 0

error_crisp_y = 0
error_fuzzy_y = 0
diff_crisp_y = 0
diff_fuzzy_y = 0
force_crisp_y = 0
force_fuzzy_y = 0
output_crisp_y = 0
output_fuzzy_y = 0

error_crisp_z = 0
error_fuzzy_z = 0
diff_crisp_z = 0
diff_fuzzy_z = 0
force_crisp_z = 0
force_fuzzy_z = 0
output_crisp_z = 0
output_fuzzy_z = 0

error_crisp_roll = 0
error_fuzzy_roll = 0
diff_crisp_roll = 0
diff_fuzzy_roll = 0
force_crisp_roll = 0
force_fuzzy_roll = 0
output_crisp_roll = 0
output_fuzzy_roll = 0

error_crisp_pitch = 0
error_fuzzy_pitch = 0
diff_crisp_pitch = 0
diff_fuzzy_pitch = 0
force_crisp_pitch = 0
force_fuzzy_pitch = 0
output_crisp_pitch = 0
output_fuzzy_pitch = 0

error_crisp_yaw = 0
error_fuzzy_yaw = 0
diff_crisp_yaw = 0
diff_fuzzy_yaw = 0
force_crisp_yaw = 0
force_fuzzy_yaw = 0
output_crisp_yaw = 0
output_fuzzy_yaw = 0

def callback_x(data):
    global error_crisp_x,error_fuzzy_x,diff_crisp_x,diff_fuzzy_x,force_crisp_x,force_fuzzy_x,output_crisp_x,output_fuzzy_x
    error_crisp_x = data.error.crisp_data
    error_fuzzy_x = data.error.fuzzy_data
    diff_crisp_x = data.diff.crisp_data
    diff_fuzzy_x = data.diff.fuzzy_data
    force_crisp_x = data.force.crisp_data
    force_fuzzy_x = data.force.fuzzy_data
    output_crisp_x = data.output.crisp_data
    output_fuzzy_x = data.output.fuzzy_data

def callback_y(data):
    global error_crisp_y,error_fuzzy_y,diff_crisp_y,diff_fuzzy_y,force_crisp_y,force_fuzzy_y,output_crisp_y,output_fuzzy_y
    error_crisp_y = data.error.crisp_data
    error_fuzzy_y = data.error.fuzzy_data
    diff_crisp_y = data.diff.crisp_data
    diff_fuzzy_y = data.diff.fuzzy_data
    force_crisp_y = data.force.crisp_data
    force_fuzzy_y = data.force.fuzzy_data
    output_crisp_y = data.output.crisp_data
    output_fuzzy_y = data.output.fuzzy_data

def callback_z(data):
    global error_crisp_z,error_fuzzy_z,diff_crisp_z,diff_fuzzy_z,force_crisp_z,force_fuzzy_z,output_crisp_z,output_fuzzy_z
    error_crisp_z = data.error.crisp_data
    error_fuzzy_z = data.error.fuzzy_data
    diff_crisp_z = data.diff.crisp_data
    diff_fuzzy_z = data.diff.fuzzy_data
    force_crisp_z = data.force.crisp_data
    force_fuzzy_z = data.force.fuzzy_data
    output_crisp_z = data.output.crisp_data
    output_fuzzy_z = data.output.fuzzy_data

def callback_roll(data):
    global error_crisp_roll,error_fuzzy_roll,diff_crisp_roll,diff_fuzzy_roll,force_crisp_roll,force_fuzzy_roll,output_crisp_roll,output_fuzzy_roll
    error_crisp_roll = data.error.crisp_data
    error_fuzzy_roll = data.error.fuzzy_data
    diff_crisp_roll = data.diff.crisp_data
    diff_fuzzy_roll = data.diff.fuzzy_data
    force_crisp_roll = data.force.crisp_data
    force_fuzzy_roll = data.force.fuzzy_data
    output_crisp_roll = data.output.crisp_data
    output_fuzzy_roll = data.output.fuzzy_data

def callback_pitch(data):
    global error_crisp_pitch,error_fuzzy_pitch,diff_crisp_pitch,diff_fuzzy_pitch,force_crisp_pitch,force_fuzzy_pitch,output_crisp_pitch,output_fuzzy_pitch
    error_crisp_pitch = data.error.crisp_data
    error_fuzzy_pitch = data.error.fuzzy_data
    diff_crisp_pitch = data.diff.crisp_data
    diff_fuzzy_pitch = data.diff.fuzzy_data
    force_crisp_pitch = data.force.crisp_data
    force_fuzzy_pitch = data.force.fuzzy_data
    output_crisp_pitch = data.output.crisp_data
    output_fuzzy_pitch = data.output.fuzzy_data

def callback_yaw(data):
    global error_crisp_yaw,error_fuzzy_yaw,diff_crisp_yaw,diff_fuzzy_yaw,force_crisp_yaw,force_fuzzy_yaw,output_crisp_yaw,output_fuzzy_yaw
    error_crisp_yaw = data.error.crisp_data
    error_fuzzy_yaw = data.error.fuzzy_data
    diff_crisp_yaw = data.diff.crisp_data
    diff_fuzzy_yaw = data.diff.fuzzy_data
    force_crisp_yaw = data.force.crisp_data
    force_fuzzy_yaw = data.force.fuzzy_data
    output_crisp_yaw = data.output.crisp_data
    output_fuzzy_yaw = data.output.fuzzy_data

def delete():
    global value_ec_x,value_ef_x,value_dc_x,value_df_x,value_fc_x,value_ff_x,value_oc_x,value_of_x
    global value_ec_y,value_ef_y,value_dc_y,value_df_y,value_fc_y,value_ff_y,value_oc_y,value_of_y
    global value_ec_z,value_ef_z,value_dc_z,value_df_z,value_fc_z,value_ff_z,value_oc_z,value_of_z
    global value_ec_roll,value_ef_roll,value_dc_roll,value_df_roll,value_fc_roll,value_ff_roll,value_oc_roll,value_of_roll
    global value_ec_pitch,value_ef_pitch,value_dc_pitch,value_df_pitch,value_fc_pitch,value_ff_pitch,value_oc_pitch,value_of_pitch
    global value_ec_yaw,value_ef_yaw,value_dc_yaw,value_df_yaw,value_fc_yaw,value_ff_yaw,value_oc_yaw,value_of_yaw
    
    del value_ec_x[0]
    del value_ef_x[0]
    del value_dc_x[0]
    del value_df_x[0]
    del value_fc_x[0]
    del value_ff_x[0]
    del value_oc_x[0]
    del value_of_x[0]
    del value_ec_y[0]
    del value_ef_y[0]
    del value_dc_y[0]
    del value_df_y[0]
    del value_fc_y[0]
    del value_ff_y[0]
    del value_oc_y[0]
    del value_of_y[0]
    del value_ec_z[0]
    del value_ef_z[0]
    del value_dc_z[0]
    del value_df_z[0]
    del value_fc_z[0]
    del value_ff_z[0]
    del value_oc_z[0]
    del value_of_z[0]
    del value_ec_roll[0]
    del value_ef_roll[0]
    del value_dc_roll[0]
    del value_df_roll[0]
    del value_fc_roll[0]
    del value_ff_roll[0]
    del value_oc_roll[0]
    del value_of_roll[0]
    del value_ec_pitch[0]
    del value_ef_pitch[0]
    del value_dc_pitch[0]
    del value_df_pitch[0]
    del value_fc_pitch[0]
    del value_ff_pitch[0]
    del value_oc_pitch[0]
    del value_of_pitch[0]
    del value_ec_yaw[0]
    del value_ef_yaw[0]
    del value_dc_yaw[0]
    del value_df_yaw[0]
    del value_fc_yaw[0]
    del value_ff_yaw[0]
    del value_oc_yaw[0]
    del value_of_yaw[0]

def append():
    global value_ec_x,value_ef_x,value_dc_x,value_df_x,value_fc_x,value_ff_x,value_oc_x,value_of_x
    global value_ec_y,value_ef_y,value_dc_y,value_df_y,value_fc_y,value_ff_y,value_oc_y,value_of_y
    global value_ec_z,value_ef_z,value_dc_z,value_df_z,value_fc_z,value_ff_z,value_oc_z,value_of_z
    global value_ec_roll,value_ef_roll,value_dc_roll,value_df_roll,value_fc_roll,value_ff_roll,value_oc_roll,value_of_roll
    global value_ec_pitch,value_ef_pitch,value_dc_pitch,value_df_pitch,value_fc_pitch,value_ff_pitch,value_oc_pitch,value_of_pitch
    global value_ec_yaw,value_ef_yaw,value_dc_yaw,value_df_yaw,value_fc_yaw,value_ff_yaw,value_oc_yaw,value_of_yaw
    value_ec_x.append(error_crisp_x)
    value_ef_x.append(error_fuzzy_x)
    value_dc_x.append(diff_crisp_x)
    value_df_x.append(diff_fuzzy_x)
    value_fc_x.append(force_crisp_x)
    value_ff_x.append(force_fuzzy_x)
    value_oc_x.append(output_crisp_x)
    value_of_x.append(output_fuzzy_x)

    value_ec_y.append(error_crisp_y)
    value_ef_y.append(error_fuzzy_y)
    value_dc_y.append(diff_crisp_y)
    value_df_y.append(diff_fuzzy_y)
    value_fc_y.append(force_crisp_y)
    value_ff_y.append(force_fuzzy_y)
    value_oc_y.append(output_crisp_y)
    value_of_y.append(output_fuzzy_y)

    value_ec_z.append(error_crisp_z)
    value_ef_z.append(error_fuzzy_z)
    value_dc_z.append(diff_crisp_z)
    value_df_z.append(diff_fuzzy_z)
    value_fc_z.append(force_crisp_z)
    value_ff_z.append(force_fuzzy_z)
    value_oc_z.append(output_crisp_z)
    value_of_z.append(output_fuzzy_z)

    value_ec_roll.append(error_crisp_roll)
    value_ef_roll.append(error_fuzzy_roll)
    value_dc_roll.append(diff_crisp_roll)
    value_df_roll.append(diff_fuzzy_roll)
    value_fc_roll.append(force_crisp_roll)
    value_ff_roll.append(force_fuzzy_roll)
    value_oc_roll.append(output_crisp_roll)
    value_of_roll.append(output_fuzzy_roll)

    value_ec_pitch.append(error_crisp_pitch)
    value_ef_pitch.append(error_fuzzy_pitch)
    value_dc_pitch.append(diff_crisp_pitch)
    value_df_pitch.append(diff_fuzzy_pitch)
    value_fc_pitch.append(force_crisp_pitch)
    value_ff_pitch.append(force_fuzzy_pitch)
    value_oc_pitch.append(output_crisp_pitch)
    value_of_pitch.append(output_fuzzy_pitch)

    value_ec_yaw.append(error_crisp_yaw)
    value_ef_yaw.append(error_fuzzy_yaw)
    value_dc_yaw.append(diff_crisp_yaw)
    value_df_yaw.append(diff_fuzzy_yaw)
    value_fc_yaw.append(force_crisp_yaw)
    value_ff_yaw.append(force_fuzzy_yaw)
    value_oc_yaw.append(output_crisp_yaw)
    value_of_yaw.append(output_fuzzy_yaw)

def plot():
    plt.ion()
    plt.subplot(6,6,1)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('x_crisp_data')
    plt.plot(value_ec_x, label = "error")
    plt.plot(value_dc_x, label = "diff")
    plt.plot(value_fc_x, label = "force")
    plt.plot(value_oc_x, label = "output")
    
    plt.subplot(6,6,2)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('x_fuzzy_data')
    plt.plot(value_ec_x, label = "error")
    plt.plot(value_dc_x, label = "diff")
    plt.plot(value_fc_x, label = "force")
    plt.plot(value_oc_x, label = "output")

    plt.subplot(6,6,3)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('y_crisp_data')
    plt.plot(value_ec_y, label = "error")
    plt.plot(value_dc_y, label = "diff")
    plt.plot(value_fc_y, label = "force")
    plt.plot(value_oc_y, label = "output")

    plt.subplot(6,6,4)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('y_fuzzy_data')
    plt.plot(value_ec_y, label = "error")
    plt.plot(value_dc_y, label = "diff")
    plt.plot(value_fc_y, label = "force")
    plt.plot(value_oc_y, label = "output")

    plt.subplot(6,6,5)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('z_crisp_data')
    plt.plot(value_ec_z, label = "error")
    plt.plot(value_dc_z, label = "diff")
    plt.plot(value_fc_z, label = "force")
    plt.plot(value_oc_z, label = "output")

    plt.subplot(6,6,6)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('z_fuzzy_data')
    plt.plot(value_ec_z, label = "error")
    plt.plot(value_dc_z, label = "diff")
    plt.plot(value_fc_z, label = "force")
    plt.plot(value_oc_z, label = "output")

    plt.subplot(6,6,7)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('roll_crisp_data')
    plt.plot(value_ec_roll, label = "error")
    plt.plot(value_dc_roll, label = "diff")
    plt.plot(value_fc_roll, label = "force")
    plt.plot(value_oc_roll, label = "output")

    plt.subplot(6,6,8)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('roll_fuzzy_data')
    plt.plot(value_ec_roll, label = "error")
    plt.plot(value_dc_roll, label = "diff")
    plt.plot(value_fc_roll, label = "force")
    plt.plot(value_oc_roll, label = "output")

    plt.subplot(6,6,9)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('pitch_crisp_data')
    plt.plot(value_ec_pitch, label = "error")
    plt.plot(value_dc_pitch, label = "diff")
    plt.plot(value_fc_pitch, label = "force")
    plt.plot(value_oc_pitch, label = "output")

    plt.subplot(6,6,10)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('pitch_fuzzy_data')
    plt.plot(value_ec_pitch, label = "error")
    plt.plot(value_dc_pitch, label = "diff")
    plt.plot(value_fc_pitch, label = "force")
    plt.plot(value_oc_pitch, label = "output")

    plt.subplot(6,6,11)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('yaw_crisp_data')
    plt.plot(value_ec_yaw, label = "error")
    plt.plot(value_dc_yaw, label = "diff")
    plt.plot(value_fc_yaw, label = "force")
    plt.plot(value_oc_yaw, label = "output")

    plt.subplot(6,6,12)
    plt.xlabel('time')
    plt.ylabel('value')
    plt.title('yaw_fuzzy_data')
    plt.plot(value_ec_yaw, label = "error")
    plt.plot(value_dc_yaw, label = "diff")
    plt.plot(value_fc_yaw, label = "force")
    plt.plot(value_oc_yaw, label = "output")

    plt.legend()
    
    plt.show()
    plt.pause(1)
    plt.clf()
    plt.ioff()
    
def add_value():
    if (len(value_ef_x) < 20):
        append()
    else:
        delete()
        append()

def run_system():
    add_value()
    plot()

if __name__ == '__main__':
        rospy.init_node('plotgraph', anonymous=True)
        rospy.Subscriber("/control/x", ControlFuzzy, callback_x)
        rospy.Subscriber("/control/y", ControlFuzzy, callback_y)
        rospy.Subscriber("/control/z", ControlFuzzy, callback_z)
        rospy.Subscriber("/control/roll", ControlFuzzy, callback_roll)
        rospy.Subscriber("/control/pitch", ControlFuzzy, callback_pitch)
        rospy.Subscriber("/control/yaw", ControlFuzzy, callback_yaw)
        rate = rospy.Rate(5)
        while(not rospy.is_shutdown() ):
            run_system()
            rate.sleep()
