within Tutorial2015.StepByStep;
model Step10_IncludingFriction
  "A controlled inverse pendulum with dry friction breaking the control loop"
  extends Modelica.Icons.Example;

  inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
    animateWorld=false,
    animateGravity=false)
    annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
  BaseComponents.SimplePlanarMechanics.Parts.Body carriage(
    m=1,
    I=0,
    animate=true)
    annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
  BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-70,-40})));
  BaseComponents.SimplePlanarMechanics.Parts.Body pendulum(
    I=0,
    m=1,
    animate=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={80,-80})));
  BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod(r={0,0.5},
      animate=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={50,-80})));
  BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute(
    animate=true,
    useFlange=true,
    w(fixed=true),
    phi(fixed=true, start=0.43633231299858)) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,-60})));
  BaseComponents.SimplePlanarMechanics.Joints.Prismatic prismatic(
    r={1,0},
    animate=true,
    useFlange=true,
    s(fixed=true),
    v(fixed=true))
    annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(
    Jr=0,
    VaNominal=12,
    useThermalPort=false,
    IaNominal=1,
    TaOperational=293.15,
    wNominal=10.471975511966,
    TaNominal=293.15,
    Ra=0.05,
    TaRef=293.15,
    alpha20a(displayUnit="1/K") = 0,
    La=0.0015,
    useSupport=false,
    Js=0)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Modelica.Mechanics.Rotational.Components.IdealGearR2T idealGearR2T(ratio=100)
    annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,0})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent
    annotation (Placement(transformation(extent={{-88,40},{-68,60}})));
  Modelica.Blocks.Continuous.PID PID(
    Ti=1e16,
    initType=Modelica.Blocks.Types.InitPID.InitialState,
    xi_start=0,
    y_start=0,
    Td=0.1,
    k=-7)   annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={50,80})));
  Modelica.Blocks.Math.Feedback feedback annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={10,80})));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Modelica.Blocks.Continuous.LimPID
                                 PD(
    Ti=1e6,
    initType=Modelica.Blocks.Types.InitPID.InitialState,
    k=2,
    yMax=1,
    Td=0.4)  annotation (Placement(transformation(extent={{40,20},{60,40}})));
  Modelica.Blocks.Sources.Step step(startTime=8)
    annotation (Placement(transformation(extent={{0,24},{12,36}})));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T=0.5, initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{20,24},{32,36}})));
  BaseComponents.Friction.IdealDryFriction idealDryFriction(S=5, R=3)
    annotation (Placement(transformation(extent={{-30,20},{-10,40}})));
  Modelica.Mechanics.Translational.Components.Fixed fixed1(s0=0)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,30})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{60,-80},{70,-80}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, rod.frame_a) annotation (Line(
      points={{20,-70},{20,-80},{40,-80}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_a, carriage.frame_a) annotation (Line(
      points={{20,-50},{20,-40},{40,-40}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_a, fixed.frame_a) annotation (Line(
      points={{-20,-40},{-60,-40}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{0,-40},{40,-40}},
      color={95,95,95},
      thickness=0.5));
  connect(dcpm.pin_an, ground.p) annotation (Line(
      points={{-76,10},{-90,10}},
      color={0,0,255}));
  connect(idealGearR2T.flangeR, dcpm.flange) annotation (Line(
      points={{-46,0},{-60,0}}));
  connect(angleSensor.flange, revolute.flange_a) annotation (Line(
      points={{80,-10},{80,-60},{30,-60}}));
  connect(ground.p, signalCurrent.p) annotation (Line(
      points={{-90,10},{-90,50},{-88,50}},
      color={0,0,255}));
  connect(signalCurrent.n, dcpm.pin_ap) annotation (Line(
      points={{-68,50},{-64,50},{-64,10}},
      color={0,0,255}));
  connect(PD.y, feedback.u2) annotation (Line(
      points={{61,30},{70,30},{70,50},{10,50},{10,72}},
      color={0,0,127}));
  connect(feedback.y, signalCurrent.i) annotation (Line(
      points={{1,80},{-78,80},{-78,57}},
      color={0,0,127}));
  connect(feedback.u1, PID.y) annotation (Line(
      points={{18,80},{39,80}},
      color={0,0,127}));
  connect(PID.u, angleSensor.phi) annotation (Line(
      points={{62,80},{80,80},{80,11}},
      color={0,0,127}));
  connect(positionSensor.s,PD. u_m) annotation (Line(
      points={{21,0},{50,0},{50,18}},
      color={0,0,127}));
  connect(firstOrder.u, step.y) annotation (Line(
      points={{18.8,30},{12.6,30}},
      color={0,0,127}));
  connect(firstOrder.y,PD. u_s) annotation (Line(
      points={{32.6,30},{38,30}},
      color={0,0,127}));
  connect(fixed1.flange, idealDryFriction.flange_a) annotation (Line(
      points={{-40,30},{-30,30}},
      color={0,127,0}));
  connect(positionSensor.flange, idealGearR2T.flangeT) annotation (Line(
      points={{0,0},{-26,0}},
      color={0,127,0}));
  connect(idealDryFriction.flange_b, idealGearR2T.flangeT) annotation (Line(
      points={{-10,30},{-10,0},{-26,0}},
      color={0,127,0}));
  connect(prismatic.flange_a, idealGearR2T.flangeT) annotation (Line(
      points={{-10,-31},{-10,0},{-26,0}},
      color={0,127,0}));
  annotation (experiment(StopTime=15,
        Interval=0.01),
    Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end Step10_IncludingFriction;
