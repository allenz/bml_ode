-- telemetry data recording from VelociRoACH dynamic simulation
-- writes to file  data-test.txt
-- by R. Fearing and Allen
-- Aug. 21, 2014
function bound(x, value)
    if x > value then
        x = value
    elseif x < -value then
        x = -value
    end
    return x
end
function normalize(x) -- return angle x in [-pi, pi)
    x = math.fmod(x,2*math.pi)
    if x >= math.pi then
        x = x - 2*math.pi
    end
    return x
end

-- obtain parameters and handles; initialize telemetry and tubes
if sim_call_type == sim_childscriptcall_initialization then
    FREQ = simGetScriptSimulationParameter(sim_handle_self, 'FREQ') * 2*math.pi -- Hz to rad/s, 3
    PHASE = simGetScriptSimulationParameter(sim_handle_self, 'PHASE') * math.pi/180 -- deg to rad, 180
    MAX_TORQUE = simGetScriptSimulationParameter(sim_handle_self, 'MAX_TORQUE') -- in N*m, 20
    MAX_VEL = simGetScriptSimulationParameter(sim_handle_self, 'MAX_VEL') -- in deg/s, 25
    
    RMotorHandle = simGetObjectHandle('MotorRF')  -- forward right motor
    RMotorHandle1 = simGetObjectHandle('MotorRB') -- back right motor
    LMotorHandle = simGetObjectHandle('MotorLF')  -- forward left motor
    LMotorHandle1 = simGetObjectHandle('MotorLB') -- back left motor
    chassisHandle = simGetObjectHandle('Chassis')
    fsensorHandleRF = simGetObjectHandle('foot0') -- right forward force sensor
    fsensorHandleRM = simGetObjectHandle('foot6') -- right middle
    fsensorHandleRB = simGetObjectHandle('foot2') -- right back
    fsensorHandleLF = simGetObjectHandle('foot3')
    fsensorHandleLM = simGetObjectHandle('foot7')
    fsensorHandleLB = simGetObjectHandle('foot5')

    simSetJointForce(RMotorHandle, MAX_TORQUE)
    simSetJointForce(RMotorHandle1, MAX_TORQUE)
    simSetJointForce(LMotorHandle, MAX_TORQUE)
    simSetJointForce(LMotorHandle1, MAX_TORQUE)
    simSetObjectFloatParameter(RMotorHandle, sim_jointfloatparam_upper_limit, MAX_VEL)
    simSetObjectFloatParameter(RMotorHandle1, sim_jointfloatparam_upper_limit, MAX_VEL)
    simSetObjectFloatParameter(LMotorHandle, sim_jointfloatparam_upper_limit, MAX_VEL)
    simSetObjectFloatParameter(LMotorHandle1, sim_jointfloatparam_upper_limit, MAX_VEL)
    simResetDynamicObject(RMotorHandle)
    simResetDynamicObject(RMotorHandle1)
    simResetDynamicObject(LMotorHandle)
    simResetDynamicObject(LMotorHandle1)
    
    consoleHandle = simAuxiliaryConsoleOpen('Telem Debugging',999,5,{40,30},nil,nil,nil,nil)
    telemFile=io.open("telem.csv","w")
    graphHandle = simGetObjectHandle('Graph')

    s = string.format('"%% V-rep data recorded %s "\n', os.date("%c"))
    s = s .. string.format('"%% FREQ = %7.3f "\n', FREQ)
    s = s .. string.format('"%% PHASE = %7.3f "\n', PHASE)
    s = s .. string.format('"%% MAX_TORQUE = %d, MAX_VEL = %d "\n', MAX_TORQUE, MAX_VEL)
    s = s .. '"% velroach+telem Vrep sim"\n'
    s = s .. string.format('"%% Motor Gains = [%6.3f, %6.3f, %6.3f, 0,0, %6.3f, %6.3f, %6.3f,0,0]"\n',
            select(2, simGetObjectFloatParameter(RMotorHandle, sim_jointfloatparam_pid_p)),
            select(2, simGetObjectFloatParameter(RMotorHandle, sim_jointfloatparam_pid_i)),
            select(2, simGetObjectFloatParameter(RMotorHandle, sim_jointfloatparam_pid_d)),
            select(2, simGetObjectFloatParameter(LMotorHandle, sim_jointfloatparam_pid_p)),
            select(2, simGetObjectFloatParameter(LMotorHandle, sim_jointfloatparam_pid_i)),
            select(2, simGetObjectFloatParameter(LMotorHandle, sim_jointfloatparam_pid_d)))
    -- TODO pid for back motors
    -- https://en.wikipedia.org/wiki/PID_controller#PID_controller_theory
    s = s .. '"% (TODO col descriptions and units) "\n'
    s = s .. 'time,LPos,RPos,LRef,RRef,LPWM,RPWM,GyroX,GyroY,GyroZ,AX,AY,AZ,LEMF,REMF,BAT,'
          .. 'x,y,z,a,b,g,refL,refL1,refR,refR1,posL,posL1,posR,posR1,velL,velL1,velR,velR1,torqueL,torqueL1,torqueR,torqueR1,'
          .. 'fRFx,fRFy,fRFz,fRMx,fRMy,fRMz,fRBx,fRBy,fRBz,fLFx,fLFy,fLFz,fLMx,fLMy,fLMz,fLBx,fLBy,fLBz\n'
    telemFile:write(s)
    simAuxiliaryConsolePrint(consoleHandle, s)

    gyroTube = simTubeOpen(0,'gyroData'..simGetNameSuffix(nil),1)
    accelTube = simTubeOpen(0,'accelerometerData'..simGetNameSuffix(nil),1)
end

-- motor control
if sim_call_type == sim_childscriptcall_actuation then
    t = simGetSimulationTime()
    refR = FREQ * t
    refL = FREQ * t + PHASE
    simSetJointTargetPosition(RMotorHandle, refR)
    simSetJointTargetPosition(RMotorHandle1, refR)
    simSetJointTargetPosition(LMotorHandle, refL)
    simSetJointTargetPosition(LMotorHandle1, refL)
end

-- Record telemetry in Duncan format
if sim_call_type == sim_childscriptcall_sensing then
    s = string.format('%8d, ', math.ceil(simGetSimulationTime() * 1e6)) -- in microseconds
    
    -- joint positions, radians
    posL = simGetJointPosition(LMotorHandle)
    posR = simGetJointPosition(RMotorHandle)
    s = s .. string.format('%8d, %8d, ',
                math.ceil(65536* (posL/(2 * math.pi))),
                math.ceil(65536* (posR/(2 * math.pi))))
    
    -- joint position targets, radians
    refL = simGetJointTargetPosition(LMotorHandle)
    refR = simGetJointTargetPosition(RMotorHandle)
    s = s .. string.format('%8d, %8d, ',
                math.ceil(65536* (refL/(2 * math.pi))),
                math.ceil(65536* (refR/(2 * math.pi))))
    simSetGraphUserData(graphHandle, 'refR', normalize(refR))
    
    -- PWM or Duty Cycle, from joint torque
    torqueL = simGetJointForce(LMotorHandle)
    torqueL1 = simGetJointForce(LMotorHandle1)
    torqueR = simGetJointForce(RMotorHandle)
    torqueR1 = simGetJointForce(RMotorHandle1)
    s = s .. string.format('%7.3f, %7.3f, ', (torqueL+torqueL1)*1e6, (torqueR+torqueR1)*1e6)

    -- Angular velocity GyroX, GyroY, GyroZ in radians/second
    -- in mpu6000.c, scale set to +-2000 degrees per second = +-32768
    data = simTubeRead(gyroTube)
    if data then
        angularRate = simUnpackFloats(data)
        s = s .. string.format('%6d, %6d, %6d,', 
            math.ceil(32768/2000*angularRate[1]*180/math.pi),
            math.ceil(32768/2000*angularRate[2]*180/math.pi),
            math.ceil(32768/2000*angularRate[3]*180/math.pi))
    else
        s = s .. string.format('0,0,0, ')
    end
    
    -- Accelerometer AX, AY, AZ
    -- in mpu6000.c, scale set to +-8g = +-32768
    data = simTubeRead(accelTube)
    if data then
        accel = simUnpackFloats(data)
        s = s .. string.format('%6d, %6d, %6d, ', 
            bound(math.ceil(32768*accel[1]/(9.8*8)), 32767),
            bound(math.ceil(32768*accel[2]/(9.8*8)), 32767),
            bound(math.ceil(32768*accel[3]/(9.8*8)), 32767))
    else
        s = s .. string.format('0,0,0, ')
    end
        
    -- BackEMF from motor velocity
    resultL, velL = simGetObjectFloatParameter(LMotorHandle, sim_jointfloatparam_velocity)
    resultR, velR = simGetObjectFloatParameter(RMotorHandle, sim_jointfloatparam_velocity)
    if resultL == 1 and resultR == 1 then
        s = s .. string.format('%7.3f, %7.3f, ', velL, velR)
    else
        s = s .. string.format('failed read velocity %d %d', resultL, resultR)
    end

    -- VBatt = 912
    s = s .. '912, '


    -- USEFUL TELEMETRY
    -- Absolute position and orientation, sim only
    a = simGetObjectPosition(chassisHandle, -1) -- Chassis x, y, z
    s = s .. string.format('%.4g, %.4g, %.4g, ', a[1], a[2], a[3])
    a = simGetObjectOrientation(chassisHandle, -1) -- Chassis Euler(a,b,g)
    s = s .. string.format('%.4g, %.4g, %.4g, ', a[1], a[2], a[3])

    -- Joint position targets (rad)
    refL1 = simGetJointTargetPosition(LMotorHandle1)
    refR1 = simGetJointTargetPosition(RMotorHandle1)
    s = s .. string.format('%7.3f, %7.3f, %7.3f, %7.3f, ', refL, refL1, refR, refR1) --TODO dedup

    -- Joint positions (rad)
    posL1 = simGetJointPosition(LMotorHandle1)
    posR1 = simGetJointPosition(RMotorHandle1)
    s = s .. string.format('%7.3f, %7.3f, %7.3f, %7.3f, ', posL, posL1, posR, posR1)

    -- Joint velocities (rad/s)
    resultL, velL1 = simGetObjectFloatParameter(LMotorHandle1, sim_jointfloatparam_velocity)
    resultR, velR1 = simGetObjectFloatParameter(RMotorHandle1, sim_jointfloatparam_velocity)
    s = s .. string.format('%7.3f, %7.3f, %7.3f, %7.3f, ', velL, velL1, velR, velR1)
    
    -- Joint torques (N*m)
    s = s .. string.format('%.4g, %.4g, %.4g, %.4g, ', torqueL, torqueL1, torqueR, torqueR1)
    
    -- Foot x/y/z forces (N)
    result, fRF, tVec = simReadForceSensor(fsensorHandleRF)
    if result and fRF then
        s = s .. string.format('%.4g, %.4g, %.4g, ', fRF[1], fRF[2], fRF[3])
    else
        s = s .. '0,0,0,'
    end
    result, fRM, tVec = simReadForceSensor(fsensorHandleRM)
    if result and fRM then
        s = s .. string.format('%.4g, %.4g, %.4g, ', fRM[1], fRM[2], fRM[3])
    else
        s = s .. '0,0,0,'
    end
    result, fRB, tVec = simReadForceSensor(fsensorHandleRB)
    if result and fRB then
        s = s .. string.format('%.4g, %.4g, %.4g, ', fRB[1], fRB[2], fRB[3])
    else
        s = s .. '0,0,0,'
    end
    result, fLF, tVec = simReadForceSensor(fsensorHandleLF)
    if result and fLF then
        s = s .. string.format('%.4g, %.4g, %.4g, ', fLF[1], fLF[2], fLF[3])
    else
        s = s .. '0,0,0,'
    end
    result, fLM, tVec = simReadForceSensor(fsensorHandleLM)
    if result and fLM then
        s = s .. string.format('%.4g, %.4g, %.4g, ', fLM[1], fLM[2], fLM[3])
    else
        s = s .. '0,0,0,'
    end
    result, fLB, tVec = simReadForceSensor(fsensorHandleLB)
    if result and fLB then
        s = s .. string.format('%.4g, %.4g, %.4g', fLB[1], fLB[2], fLB[3])
    else
        s = s .. '0,0,0'
    end
    s = s .. '\n'

    simAuxiliaryConsolePrint(consoleHandle, s)
    telemFile:write(s)
end

if (sim_call_type==sim_childscriptcall_cleanup) then 
    telemFile:close()
end


-- Foot x/y/z forces, in newton
    -- result, fRF, tVec = simReadForceSensor(fsensorHandleRF)
    -- if not (result and fRF) then fRF = {0,0,0} end
    -- result, fRM, tVec = simReadForceSensor(fsensorHandleRM)
    -- if not (result and fRM) then fRM = {0,0,0} end
    -- result, fRB, tVec = simReadForceSensor(fsensorHandleRB)
    -- if not (result and fRB) then fRB = {0,0,0} end
    -- result, fLF, tVec = simReadForceSensor(fsensorHandleLF)
    -- if not (result and fLF) then fLF = {0,0,0} end
    -- result, fLM, tVec = simReadForceSensor(fsensorHandleLM)
    -- if not (result and fLM) then fLM = {0,0,0} end
    -- result, fLB, tVec = simReadForceSensor(fsensorHandleLB)
    -- if not (result and fLB) then fLB = {0,0,0} end

    -- s = s .. string.format('%.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, ' ..
    --                        '%.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g, %.4g\n',
    --                        fRF[1], fRF[2], fRF[3], fRM[1], fRM[2], fRM[3], fRB[1], fRB[2], fRB[3],
    --                        fLF[1], fLF[2], fLF[3], fLM[1], fLM[2], fLM[3], fLB[1], fLB[2], fLB[3])
