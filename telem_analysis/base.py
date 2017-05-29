import numpy as np
import pandas as pd

# Basic elements common to all notebooks
# by R. Fearing and Allen

# Constants
legScale = 2*np.pi/2**16 # 16 bit to radian
gyroScale = 2000*np.pi/180/2**15 # +-32768 data = +-2000 deg/s, to rad/s
xlScale = 8*9.8/2**15 # +-32768 data = +-8g

vref = 3.3 # for voltage conversion
vdivide = 3.7/2.7  # for battery scaling
vgain = 15.0/47.0  # gain of differential amplifier
RMotor = 3.3   # resistance for SS7-3.3 ** need to check **
Kt = 1.41 #  motor toriqe constant mN-m/A  ** SS7-3.3 **

width, height = 16, 8 # plot dim

plot = None

# Note: AngleX / a is pitch, AngleY / b = roll, g = yaw

def process(file):
    '''Returns dataframe containing processed telemetry data.'''
    # with a sane format, we could just use: t = pd.read_csv(file, comment='%')
    with open(file) as f:
        header = [f.readline() for _ in range(8)]
    names = header[-1].strip().lstrip('"%').rstrip('"').strip().replace(' | ',',').split(',')
    t = pd.read_csv(file, skiprows=8, header=None, names=names)
    
    t.time *= 1e-3 # convert to ms
    if 'posL' not in t:
        t.rename(columns={'LPos': 'posL', 'RPos': 'posR', 'LRef': 'refL', 'RRef': 'refR'}, inplace=True)
        t[['posL', 'posR', 'refL', 'refR']] *= legScale
    t[['posL', 'posR', 'refL', 'refR']] %= 2.0*np.pi
    if 'dcL' in t:
        t.rename(columns={'dcL': 'DCL', 'dcR': 'DCR'}, inplace=True)
    elif 'LPWM' in t:
        t.rename(columns={'LPWM': 'DCL', 'RPWM': 'DCR'}, inplace=True)
    t[['DCR','DCL']] *= 100/4096 # torque, as a percentage
    if 'GyX' in t:
        t.rename(columns={'GyX': 'GyroX', 'GyY': 'GyroY', 'GyZ': 'GyroZ'}, inplace=True)
    t[['GyroX','GyroY','GyroZ']] *= gyroScale
    t[['AX','AY','AZ']] *= xlScale
    # BackEMF A/D t is 10 bits, Vref+ = AVdd = 3.3V, VRef- = AVss = 0.0V
    # BEMF volts = (15K)/(47K) * Vm + vref/2 - pidObjs[i].inputOffset
    t[['LEMF','REMF']] *= vref/1024/vgain # scale A/D to 0 to 3.3 V range and undo diff amp gain
    if 'BAT' in t:
        t.rename(columns={'BAT': 'VBatt'}, inplace=True)
    t.VBatt *= vdivide*vref/1023.0 # in volts
    
    # i_m = (VBatt - BEMF)/R, i_m_avg = i_m x duty cycle
    t['CurrentL'] = (np.abs(t.DCL)/100.0)*(np.sign(t.DCL)*t.VBatt - t.LEMF)/RMotor
    t['CurrentR'] = (np.abs(t.DCR)/100.0)*(np.sign(t.DCR)*t.VBatt - t.REMF)/RMotor
    t[['TorqueL','TorqueR']] = Kt * t[['CurrentL','CurrentR']] # Tau = Kt i_m_avg
    t[['PowerL','PowerR']] = np.abs(t[['CurrentL','CurrentR']].multiply(t.VBatt, axis=0)) # P = V_m i_m_avg
    t[['AngleY','AngleZ']] = t[['GyroY','GyroZ']].cumsum()/100 # sampling rate: 10 ms -> 100 Hz

    # total forces, sim only
    if 'fRFx' in t:
        t['fRF'] = np.sqrt(np.square(t[['fRFx','fRFy','fRFz']]).sum(axis=1))
        t['fRM'] = np.sqrt(np.square(t[['fRMx','fRMy','fRMz']]).sum(axis=1))
        t['fRB'] = np.sqrt(np.square(t[['fRBx','fRBy','fRBz']]).sum(axis=1))
        t['fLF'] = np.sqrt(np.square(t[['fLFx','fLFy','fLFz']]).sum(axis=1))
        t['fLM'] = np.sqrt(np.square(t[['fLMx','fLMy','fLMz']]).sum(axis=1))
        t['fLB'] = np.sqrt(np.square(t[['fLBx','fLBy','fLBz']]).sum(axis=1))
        t['fR'] = t[['fRF','fRM','fRB']].sum(axis=1)
        t['fL'] = t[['fLF','fLM','fLB']].sum(axis=1)

    return t

def setPlot(plt):
    global plot
    plot = plt

def tWidth(t, ax=None, ms=3000, pad=0):
    '''Set plot time range'''
    if not ax:
        ax = plot
    if ms != 0 and t.time.iloc[-1] > ms:
        lim = [t.time.iloc[-1]//2 - ms//2 - pad, t.time.iloc[-1]//2 + ms//2 + pad]
    else:
        lim = [t.time[0] - pad, t.time.iloc[-1] + pad]
    try:
        ax.xlim(lim) # plot object
    except AttributeError:
        ax.set_xlim(lim) # axes object

def pler(axes, xlim=None):
    plot.xlabel('time (ms)')
    axes.axhline(color='m')
    if xlim:
        try:
            axes.xlim(xlim) # plot object
        except AttributeError:
            axes.set_xlim(xlim) # axes object

def plotBasic(t, limits=None):
    '''Straightforward plot of telemetry data'''
    fig = plot.figure(figsize = (width, height))

    # actual and commanded leg position
    axes = plot.subplot(3,2,1)
    plot.plot(t.time, t.posL, 'b-', t.time, t.posR, 'g-',
              t.time, t.refL, 'b--', t.time, t.refR, 'g--')
    plot.ylabel('Leg Position')
    plot.legend(['posL','posR','refL','refR'])
    pler(axes)

    # Motor PWM 
    axes = plot.subplot(3,2,2)
    plot.plot(t.time, t.DCL, 'k', t.time, t.DCR, 'b--')
    plot.ylabel('Duty Cycle (%)')
    plot.legend(['DCL', 'DCR'])
    pler(axes)

    # gyro t
    axes = plot.subplot(3,2,3)
    plot.plot(t.time, t.GyroX, 'r-', t.time, t.GyroY, 'g-', t.time, t.GyroZ, 'b-')
    plot.ylabel('Gyro (rad/s)')
    plot.legend(['GyroX','GyroY','GyroZ'])
    pler(axes)

    # accelerometer t
    axes = plot.subplot(3,2,4)
    plot.plot(t.time, t.AX, 'r-', t.time, t.AY, 'g-', t.time, t.AZ, 'b-')
    plot.ylabel('Accel $m s^{-2}$')
    plot.legend(['AX', 'AY', 'AZ'])
    pler(axes)

    #back EMF
    axes = plot.subplot(3,2,5)
    plot.plot(t.time, t.LEMF, 'b', t.time, t.REMF, 'g')
    plot.ylabel('Back EMF (V)')
    plot.legend(['LEMF', 'REMF'])
    pler(axes)

    #battery voltage
    axes = plot.subplot(3,2,6)
    plot.plot(t.time, t.VBatt)
    plot.ylabel('Batt (V)')
    pler(axes)

def plotPAT(t, xlim=[0,3000]):
    # def pler(axes, limits=None):
    #     plot.xlabel('time [ms]')
    #     axes.axhline(linewidth=1, color='m')
    #     if limits:
    #         plot.xlim(limits)
    fig = plot.figure(figsize = (9, 8))

    # Actual and commanded leg position
    plot.subplot(3,1,1)
    plot.plot(t.time,t.posR, 'k')
    plot.plot(t.time,t.posL, 'b')
    plot.plot(t.time,t.refR, 'k--')
    plot.plot(t.time,t.refL, 'b-.')
    plot.xlabel('Time (ms)')
    plot.ylabel('Leg Position (m)')
    plot.legend(['RPos','LPos','Rref','Lref'])
    ax = fig.add_subplot(3,1,1)
    pler(ax, xlim)

    # # Gyro data
    # plot.subplot(3,1,2)
    # plot.plot(t.time,t.GyroY, 'g.')
    # plot.plot(t.time,t.GyroZ, 'b')
    # tWidth(t, pad=500)
    # plot.xlabel('t.time (ms)')
    # plot.ylabel('Gyro (rad/s)')
    # plot.legend(['Y', 'Z'])
    # ax = fig.add_subplot(3,1,2)
    # ax.axhline(linewidth=1, color='m')

    # Gyro data
    plot.subplot(3,1,2)
    plot.plot(t.time,t.AngleZ, 'k')
    plot.xlabel('Time (ms)')
    plot.ylabel('Angle (rad)')
    ax = fig.add_subplot(3,1,2)
    pler(ax, xlim)

    # Torque
    plot.subplot(3,1,3)
    if 'torqueR' in t:
        plot.plot(t.time,(t.torqueR+t.torqueR1), 'k', t.time,(t.torqueL+t.torqueL1), 'b--')
    else:
        plot.plot(t.time, t.TorqueR, 'k', t.time, t.TorqueL, 'b--')
    plot.xlabel('Time [ms]')
    plot.ylabel('Torque (N-m)')
    plot.legend(['Right', 'Left'])
    ax = fig.add_subplot(3,1,3)
    pler(ax, xlim)

def plotFXYZ(t):
    plot.plot(t.time, -t.fLFx, 'r', t.time, -t.fLFy, 'g', t.time, t.fLFz, 'b')
    tWidth(t)
    plot.xlabel('Time (ms)')
    plot.ylabel('Force (N)')
    plot.legend(['Fx','Fy','Fz'], loc=2)
    print('Sample force sensor readout; total force = norm(Fx, Fy, Fz)')

def plotForceLR(t, xlim=[0,3000]):
    fig, (ax, bx, cx) = plot.subplots(3,1, sharey=True, figsize = (9, 8))
    plot.xlabel('Time (ms)')
    bx.set_ylabel('Angle (rad/20) / Force (N)')
    plot.ylim([0, 0.6])

    ax.plot(t.time, t.b/20, 'k', t.time, t.fRF, 'r', t.time, t.fRM, 'g', t.time, t.fRB, 'b')
    pler(ax, xlim)
    ax.legend(['Roll (R+)','Force RF','Force RM', 'Force RB'], loc=2)
    ax.set_title('Right side forces')

    bx.plot(t.time, t.b/20, 'k', t.time, t.fLF, 'r', t.time, t.fLM, 'g', t.time, t.fLB, 'b')
    pler(bx, xlim)
    bx.set_title('Left side forces')
    bx.legend(['Roll','Force LF','Force LM', 'Force LB'], loc=2)

    cx.plot(t.time, t.b/20, 'k', t.time, t.fL, 'b', t.time, t.fR, 'g')
    pler(cx, xlim)
    cx.set_title('Left vs. right force')
    cx.legend(['Roll','Force L','Force R'], loc=2)
    print('Alternating tripod gait: roll right (positive) = LM force. Roll left = LF and LB force')

def plotRollSteer(t):
    maxAngle = np.max(t.posR) - np.min(t.posR) # get max change in angle
    # print 'rightLegPos', rightLegPos
    print('maxAngle=', maxAngle)
    maxCycle = np.int(maxAngle/np.pi)  # half steps
    print('maxCycle=', maxCycle)
    deltaYaw = np.zeros(len(t.posR)) # record change in yaw angle at each step
    yawStep = np.zeros(len(t.posR))
    xticks = np.zeros(maxCycle+4)  # hold tick locations
    xticks = np.arange(0,maxCycle+4)  # non-zero place holder for xticks
    minThresh =  0.1  ## left angle is decreasing and cycling to 2 pi
    maxThresh = 1
    minFound = False
    yawHold = 0.0
    j = 0
    for i in range(0,len(t)):
        pos = t.posR[i] % (2.0*np.pi)   # get right leg position mod 2 pi
        if not minFound and (pos > (np.pi-maxThresh)) and (pos < (np.pi-minThresh)):
            minFound = True
        if minFound and (pos > (np.pi+minThresh)):
            xticks[j] = i
            j = j+1
            yawHold = t.AngleZ[i]  # save yaw value at end of leg cycle
            print('i=%d yawHold =%6.3f' %(i,yawHold))
            minFound = False
            print('i=%d, rightLegPos %6.3f' %(i, (t.posR[i] % (2.0*np.pi))))
        yawStep[i] = yawHold 
        deltaYaw[i] = t.AngleZ[i] -yawHold  # shift value for next step
     #   print 'i,leftLegPos',i,(leftLegPos[i] % (2.0*np.pi))
    print('yaw max =%6.3f' %(t.AngleZ[i]))

    min= 0
    max = 2500
    max=np.min([max, len(t.posR)])
    fig = plot.figure(figsize = (8, 9))
    # actual and commanded leg position
    plot.subplot(3,1,1)
    # actual and commanded leg position
    plot.plot(t.time[min:max],t.posR[min:max]% (2.0*np.pi),'g--')
    plot.plot(t.time[min:max],2.0*np.pi-t.posL[min:max]% (2.0*np.pi),'b')
    plot.ylabel('Leg Position')
    plot.legend(['RPos','LPos'],bbox_to_anchor=(0.85, 1), loc=2, borderaxespad=0.)
    ax = fig.add_subplot(3,1,1)
    ax.set_yticks(np.round(np.linspace(0,2,3)*np.pi,2))
    ax.yaxis.grid() #horiz lines
    ax.set_xticks(t.time[np.round(xticks,1)])
    ax.xaxis.grid() #vertical lines

    plot.subplot(3,1,2)
    # Total Angle in radians
    #plot.plot(time,AngleY,'k')
    plot.plot(t.time[min:max],t.AngleZ[min:max], 'g')
    plot.ylabel('Angle (rad)')
    plot.legend(['Gz'],bbox_to_anchor=(0.85, 1), loc=2, borderaxespad=0.)
    ax = fig.add_subplot(3,1,2)
    ax.axhline(linewidth=1, color='m')
    ax.set_xticks(t.time[np.round(xticks,1)])
    ax.xaxis.grid() #vertical lines

    plot.subplot(3,1,3)
    plot.xlabel('time (sec)')
    # Total Angle in radians
    plot.plot(t.time[min:max],t.GyroY[min:max], 'g') # gyro angle vs phase difference
    plot.ylabel('Angle Est.(rad)')
    plot.legend(['$\Delta Gz$'], bbox_to_anchor=(0.85, 1), loc=2, borderaxespad=0.)
    ax = fig.add_subplot(3,1,3)
    ax.axhline(linewidth=1, color='m')
    ax.set_xticks(t.time[np.round(xticks,1)])
    ax.xaxis.grid() #vertical lines
