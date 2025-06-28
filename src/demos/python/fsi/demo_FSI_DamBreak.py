import pychrono as chrono
import pychrono.fsi as fsi
import pychrono.vsg as vsg
import os
import pychrono.irrlicht as chronoirr

def main():
    input_json = "demo_FSI_DamBreak_Explicit.json"
    t_end = 1.0
    verbose = True
    output = True
    output_fps = 20
    render = True
    render_fps = 100
    snapshots = False
    ps_freq = 1

    # Dimension of the space domain
    bxDim = 12.0
    byDim = 1.0
    bzDim = 8.0

    # Dimension of the fluid domain
    fxDim = 4.0
    fyDim = 1.0
    fzDim = 4.0
    
    # Create a physics system and an FSI system
    sysMBS = chrono.ChSystemSMC()
    sysSPH = fsi.ChFsiFluidSystemSPH()
    sysFSI = fsi.ChFsiSystemSPH(sysMBS, sysSPH)

    sysFSI.SetVerbose(verbose)
    
    # Use the specified input JSON file
    sysSPH.ReadParametersFromFile(input_json)
    
    # Set frequency of proximity search
    sysSPH.SetNumProximitySearchSteps(ps_freq)

    # Set the shifting method
    sysSPH.SetShiftingMethod(fsi.ShiftingMethod_XSPH)
    sysSPH.SetShiftingXSPHParameters(0.5)
    
    # Set up the periodic boundary condition (only in Y direction)
    initSpace0 = sysSPH.GetInitialSpacing()
    cMin = chrono.ChVector3d(-bxDim / 2 - 10 * initSpace0, -byDim / 2 - initSpace0 / 2, -2 * bzDim)
    cMax = chrono.ChVector3d(+bxDim / 2 + 10 * initSpace0, +byDim / 2 + initSpace0 / 2, +2 * bzDim)
    sysSPH.SetComputationalDomain(chrono.ChAABB(cMin, cMax), fsi.BC_Y_PERIODIC)

    # Create Fluid region and discretize with SPH particles
    boxCenter = chrono.ChVector3d(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2)
    boxHalfDim = chrono.ChVector3d(fxDim / 2 - initSpace0, fyDim / 2, fzDim / 2 - initSpace0)

    # Use a chrono sampler to create a bucket of points
    sampler = chrono.ChGridSamplerd(initSpace0)
    points = sampler.SampleBox(boxCenter, boxHalfDim)

    # Add fluid particles from the sampler points to the FSI system
    numPart = len(points)
    gz = abs(sysSPH.GetGravitationalAcceleration().z)
    for i in range(numPart):
        # Calculate the pressure of a steady state (p = rho*g*h)
        pre_ini = sysSPH.GetDensity() * gz * (-points[i].z + fzDim)
        rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed())
        sysSPH.AddSPHParticle(points[i], rho_ini, pre_ini, sysSPH.GetViscosity())

    # Create container and attach BCE SPH particles
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    sysMBS.AddBody(ground)

    sysSPH.AddBoxContainerBCE(ground,
                              chrono.ChFramed(chrono.ChVector3d(0, 0, bzDim / 2), chrono.QUNIT),
                              chrono.ChVector3d(bxDim, byDim, bzDim),
                              chrono.ChVector3i(2, 0, 2))

    # Complete construction of the FSI system
    sysFSI.Initialize()
    
    # Output directories
    out_dir = chrono.GetChronoOutputPath() + "FSI_Dam_Break/"
    
    problem_name = sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphIntegrationSchemeString() + "_ps" + str(ps_freq)
    out_dir = os.path.join(out_dir, problem_name)
    
    try:
        os.makedirs(out_dir, exist_ok=True)
        if output:
            os.makedirs(os.path.join(out_dir, "particles"), exist_ok=True)
        if snapshots:
            os.makedirs(os.path.join(out_dir, "snapshots"), exist_ok=True)
    except OSError as e:
        print(f"Error creating directory {out_dir}: {e}")
        return 1
        
    vis = None
    if render:
        col_callback = fsi.ParticleVelocityColorCallback(0, 5.0)
        
        visFSI = fsi.ChFsiVisualizationVSG(sysFSI)
        visFSI.EnableFluidMarkers(True)
        visFSI.EnableBoundaryMarkers(True)
        visFSI.EnableRigidBodyMarkers(False)
        visFSI.SetSPHColorCallback(col_callback)

        visVSG = vsg.ChVisualSystemVSG()
        visVSG.AttachPlugin(visFSI)
        visVSG.AttachSystem(sysMBS)
        visVSG.SetWindowTitle("Dam Break")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.AddCamera(chrono.ChVector3d(0, -12 * byDim, 0.5 * bzDim), chrono.ChVector3d(0, 0, 0.4 * bzDim))
        visVSG.SetLightIntensity(0.9)
        visVSG.SetLightDirection(-chrono.CH_PI_2, chrono.CH_PI / 6)

        visVSG.Initialize()
        vis = visVSG;


    # Start the simulation
    dT = sysFSI.GetStepSizeCFD()
    time = 0
    sim_frame = 0
    out_frame = 0
    render_frame = 0

    timer = chrono.ChTimer()
    timer.start()
    while (time < t_end):
        # Save simulation data
        if output and time >= out_frame / output_fps:
            print("------- OUTPUT")
            sysSPH.SaveParticleData(os.path.join(out_dir, "particles"))
            out_frame += 1

        # Render FSI system
        if (render and time >= render_frame / render_fps):
            if not vis.Run():
                break
            vis.Render()


            render_frame+=1
        

        # Call the FSI solver
        sysFSI.DoStepDynamics(dT)

        time += dT
        sim_frame += 1
        
    timer.stop()
    print(f"End Time: {t_end}")
    print(f"\nSimulation time: {timer.GetTimeSeconds()} seconds\n")

    return 0


if __name__ == '__main__':
    main()
