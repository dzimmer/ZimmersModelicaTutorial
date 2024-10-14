within Tutorial2024.StepByStep;
model Step12_WithCooling
  "Electrically Actuated Crane Crab with Position Control and Friction"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-60,-80},{-40,-60}})));
  PlanarMechanics.Parts.Body carriage(m=1, I=0.001)
    annotation (Placement(transformation(extent={{8,-30},{28,-10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-60,-20})));
  PlanarMechanics.Parts.Body pendulum(m=0.5, I=0.001)
    annotation (Placement(transformation(extent={{42,-80},{62,-60}})));
  PlanarMechanics.Parts.FixedTranslation rod(r={0,0.5})
    annotation (Placement(transformation(extent={{-2,-80},{18,-60}})));
  PlanarMechanics.Joints.Revolute revolute1(useFlange=true, phi(start=
          0.26179938779915, fixed=true))
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-12,-50})));
  PlanarMechanics.Joints.Prismatic prismatic(useFlange=true, r={1,0})
    annotation (Placement(transformation(extent={{-32,-10},{-12,-30}})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(
    Jr=0,
    VaNominal=12,
    useThermalPort=true,
    Ra=0.05,
    alpha20a(displayUnit="1/K") = 0,
    La=0.0015,
    useSupport=false,
    Js=0,
    TaOperational=293.15,
    IaNominal=1,
    wNominal(displayUnit="rad/s") = 100,
    TaNominal=293.15,
    TaRef=293.15,
    frictionParameters(PRef=1),
    coreParameters(PRef=1),
    strayLoadParameters(PRef=1),
    brushParameters(V=0.1))
    annotation (Placement(transformation(extent={{-132,20},{-112,40}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-160,20},{-140,40}})));
  Modelica.Mechanics.Translational.Components.IdealGearR2T idealGearR2T(ratio=100)
    annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(extent={{8,-60},{28,-40}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-116,82})));
  Modelica.Blocks.Continuous.PID PID_phi(
    k=-54.55,
    Ti=1e10,
    Td=0.0685)
            annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={106,84})));
  Modelica.Blocks.Continuous.PID PID_s(
    k=11.64,
    Ti=1e10,
    Td=1.035)
            annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={74,20})));
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{32,30},{52,10}})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-10,82})));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor
    annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  AuxiliaryComponents.IdealDryFriction idealDryFriction(S=6, R=2)
    annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  AuxiliaryComponents.ThermalCollector thermalCollector
    annotation (Placement(transformation(extent={{-132,-12},{-112,8}})));
  Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-116,54})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=2) annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-60,82})));
  ThermofluidStream.Boundaries.Source source(
    redeclare package Medium = ThermofluidStream.Media.myMedia.Air.DryAirNasa,
    p0_par=101000,
    T0_par=293.15)
    annotation (Placement(transformation(extent={{-218,-50},{-198,-30}})));
  ThermofluidStream.Boundaries.Sink sink(redeclare package Medium =
        ThermofluidStream.Media.myMedia.Air.DryAirNasa, p0_par=100000)
    annotation (Placement(transformation(extent={{-78,-50},{-58,-30}})));
  ThermofluidStream.Processes.ThermalConvectionPipe thermalConvectionPipe(
    redeclare package Medium = ThermofluidStream.Media.myMedia.Air.DryAirNasa,
    l=0.05,
    r=0.01)
    annotation (Placement(transformation(extent={{-132,-30},{-112,-50}})));
  inner ThermofluidStream.DropOfCommons dropOfCommons(m_flow_reg=0.0001)
    annotation (Placement(transformation(extent={{-152,-80},{-132,-60}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm(
    redeclare package Medium = ThermofluidStream.Media.myMedia.Air.DryAirNasa,
    temperatureUnit="degC",
    massFlowUnit="(g/s)")
    annotation (Placement(transformation(extent={{-104,-40},{-84,-20}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm1(
    redeclare package Medium = ThermofluidStream.Media.myMedia.Air.DryAirNasa,
    temperatureUnit="degC",
    massFlowUnit="(g/s)")
    annotation (Placement(transformation(extent={{-160,-40},{-140,-20}})));
  ThermofluidStream.Processes.FlowResistance flowResistance(
    redeclare package Medium = ThermofluidStream.Media.myMedia.Air.DryAirNasa,
    redeclare function pLoss =
        ThermofluidStream.Processes.Internal.FlowResistance.referencePressureLoss
        (
        dp_ref=1000,
        m_flow_ref=0.001,
        rho_ref=1,
        dp_function=ThermofluidStream.Processes.Internal.ReferencePressureDropFunction.linear),
    l=0.05,
    r=0.01)
    annotation (Placement(transformation(extent={{-188,-50},{-168,-30}})));

  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=0.1, T(
        fixed=true))
    annotation (Placement(transformation(extent={{-154,-4},{-134,16}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid(
    amplitude=1,
    rising=2,
    width=3,
    falling=2,
    period=10,
    offset=0,
    startTime=3)
    annotation (Placement(transformation(extent={{-12,38},{8,58}})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{18,-70},{42,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_b, rod.frame_a) annotation (Line(
      points={{-12,-60},{-12,-70},{-2,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_a, carriage.frame_a) annotation (Line(
      points={{-12,-40},{-12,-20},{8,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame, prismatic.frame_a) annotation (Line(
      points={{-50,-20},{-32,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{-12,-20},{8,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(ground.p, dcpm.pin_an)
    annotation (Line(points={{-150,40},{-128,40}},
                                                 color={0,0,255}));
  connect(revolute1.frame_a, prismatic.frame_b) annotation (Line(
      points={{-12,-40},{-12,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(dcpm.flange, idealGearR2T.flangeR)
    annotation (Line(points={{-112,30},{-52,30}},color={0,0,0}));
  connect(idealGearR2T.flangeT, prismatic.flange_a) annotation (Line(points={{-32,30},
          {-16,30},{-16,-6},{-22,-6},{-22,-10}},
                                       color={0,127,0}));
  connect(ground.p, signalCurrent.p) annotation (Line(points={{-150,40},{-150,
          96},{-116,96},{-116,92}}, color={0,0,255}));
  connect(angleSensor.flange, revolute1.flange_a)
    annotation (Line(points={{8,-50},{-2,-50}}, color={0,0,0}));
  connect(angleSensor.phi, PID_phi.u) annotation (Line(points={{29,-50},{128,
          -50},{128,84},{118,84}}, color={0,0,127}));
  connect(add.u2, PID_phi.y) annotation (Line(points={{2,88},{48,88},{48,84},{
          95,84}}, color={0,0,127}));
  connect(add.u1, PID_s.y) annotation (Line(points={{2,76},{2,64},{92,64},{92,
          20},{85,20}}, color={0,0,127}));
  connect(idealGearR2T.flangeT, positionSensor.flange) annotation (Line(points=
          {{-32,30},{-16,30},{-16,20},{-10,20}}, color={0,127,0}));
  connect(PID_s.u, feedback.y)
    annotation (Line(points={{62,20},{51,20}}, color={0,0,127}));
  connect(positionSensor.s, feedback.u1)
    annotation (Line(points={{11,20},{34,20}}, color={0,0,127}));
  connect(prismatic.support, idealDryFriction.flange_a) annotation (Line(points
        ={{-28,-10},{-62,-10},{-62,0},{-52,0}}, color={0,127,0}));
  connect(idealDryFriction.flange_b, prismatic.flange_a)
    annotation (Line(points={{-32,0},{-22,0},{-22,-10}}, color={0,127,0}));
  connect(dcpm.thermalPort, thermalCollector.thermalPort)
    annotation (Line(points={{-122,20},{-122,8}}, color={191,0,0}));
  connect(signalCurrent.n, powerSensor.pc)
    annotation (Line(points={{-116,72},{-116,64}}, color={0,0,255}));
  connect(dcpm.pin_ap, powerSensor.nc)
    annotation (Line(points={{-116,40},{-116,44}}, color={0,0,255}));
  connect(powerSensor.pv, dcpm.pin_ap) annotation (Line(points={{-106,54},{-98,
          54},{-98,40},{-116,40}}, color={0,0,255}));
  connect(powerSensor.nv, signalCurrent.p) annotation (Line(points={{-126,54},{
          -150,54},{-150,96},{-116,96},{-116,92}}, color={0,0,255}));
  connect(add.y, limiter.u)
    annotation (Line(points={{-21,82},{-48,82}}, color={0,0,127}));
  connect(limiter.y, signalCurrent.i)
    annotation (Line(points={{-71,82},{-104,82}}, color={0,0,127}));
  connect(thermalCollector.port_a, thermalConvectionPipe.heatPort)
    annotation (Line(points={{-122,-12},{-122,-30}}, color={191,0,0}));
  connect(thermalConvectionPipe.outlet, multiSensor_Tpm.inlet) annotation (Line(
      points={{-112,-40},{-104,-40}},
      color={28,108,200},
      thickness=0.5));
  connect(thermalConvectionPipe.inlet, multiSensor_Tpm1.outlet) annotation (
      Line(
      points={{-132,-40},{-140,-40}},
      color={28,108,200},
      thickness=0.5));
  connect(multiSensor_Tpm.outlet, sink.inlet) annotation (Line(
      points={{-84,-40},{-78,-40}},
      color={28,108,200},
      thickness=0.5));
  connect(flowResistance.inlet, source.outlet) annotation (Line(
      points={{-188,-40},{-198,-40}},
      color={28,108,200},
      thickness=0.5));
  connect(flowResistance.outlet, multiSensor_Tpm1.inlet) annotation (Line(
      points={{-168,-40},{-160,-40}},
      color={28,108,200},
      thickness=0.5));
  connect(heatCapacitor.port, thermalCollector.port_a) annotation (Line(points=
          {{-144,-4},{-132,-4},{-132,-12},{-122,-12}}, color={191,0,0}));
  connect(trapezoid.y, feedback.u2)
    annotation (Line(points={{9,48},{42,48},{42,28}}, color={0,0,127}));
  annotation (experiment(
      StopTime=60,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-220,-100},{140,100}})),
    Icon(coordinateSystem(extent={{-220,-100},{140,100}})));
end Step12_WithCooling;
