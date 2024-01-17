function handleUI(p)
    local s=sim.getObjectSelection()
    if s and #s>0 and s[#s]==model then
        if not ui then
            local xml =[[<ui title="Gyro sensor" closeable="false" placement="relative" position="50,-50" layout="form">
                    <label text="x gyro:" />
                    <label id="1" text="-" />
                    <label text="y gyro:" />
                    <label id="2" text="-" />
                    <label text="z gyro:" />
                    <label id="3" text="-" />
            </ui>]]
            ui=simUI.create(xml)
        end
        simUI.setLabelText(ui,1,string.format("%.3f",p[1]))
        simUI.setLabelText(ui,2,string.format("%.3f",p[2]))
        simUI.setLabelText(ui,3,string.format("%.3f",p[3]))
    else
        if ui then
            simUI.destroy(ui)
            ui=nil
        end
    end
end

function sysCall_init() 
    model=sim.getObjectHandle(sim.handle_self)
    ref=sim.getObjectHandle('GyroSensor_reference')
    oldTransformationMatrix=sim.getObjectMatrix(ref,-1)
    lastTime=sim.getSimulationTime()
end

function sysCall_sensing() 
    local gyroData=(sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(ref,-1)))
    gyroData[1] = gyroData[1]--*(180/3.14159)
    gyroData[2] = gyroData[2]--*(180/3.14159)
    gyroData[3] = gyroData[3]--*(180/3.14159)
    handleUI(gyroData)
    sim.setFloatSignal('gyroX',gyroData[1])
    sim.setFloatSignal('gyroY',gyroData[2])
    sim.setFloatSignal('gyroZ',gyroData[3])
    
    
    -- To read data from this gyro sensor in another script, use following code:
    --
    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    -- data=sim.tubeRead(gyroCommunicationTube)
    -- if (data) then
    --     angularVariations=sim.unpackFloatTable(data)
    -- end
    --
    -- If the script in which you read the gyro sensor has a different suffix than the gyro suffix,
    -- then you will have to slightly adjust the code, e.g.:
    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData#') -- if the gyro script has no suffix
    -- or
    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData#0') -- if the gyro script has a suffix 0
    -- or
    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData#1') -- if the gyro script has a suffix 1
    -- etc.
    --
    --
    -- You can of course also use global variables (not elegant and not scalable), e.g.:
    -- In the gyro script:
    -- sim.setFloatSignal('gyroX',angularVariation[1])
    -- sim.setFloatSignal('gyroY',angularVariation[2])
    -- sim.setFloatSignal('gyroZ',angularVariation[3])
    --
    -- And in the script that needs the data:
    -- angularVariationX=sim.getFloatSignal('gyroX')
    -- angularVariationY=sim.getFloatSignal('gyroY')
    -- angularVariationZ=sim.getFloatSignal('gyroZ')
    --
    -- In addition to that, there are many other ways to have 2 scripts exchange data. Check the documentation for more details
end 
