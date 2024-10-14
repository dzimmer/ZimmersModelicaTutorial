within OptimizationTutorial2024.Examples;
model Step11_Solution
  "Electrically Actuated Crane Crab with Position Control and Friction"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-122,-80},{-102,-60}})));
  PlanarMechanics.Parts.Body carriage(m=1, I=0.001)
    annotation (Placement(transformation(extent={{-22,-30},{-2,-10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-102,-20})));
  PlanarMechanics.Parts.Body pendulum(m=0.5, I=0.001)
    annotation (Placement(transformation(extent={{12,-80},{32,-60}})));
  PlanarMechanics.Parts.FixedTranslation rod(r={0,0.5})
    annotation (Placement(transformation(extent={{-32,-80},{-12,-60}})));
  PlanarMechanics.Joints.Revolute revolute1(useFlange=true, phi(start=
          0.26179938779915, fixed=true))
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-42,-50})));
  PlanarMechanics.Joints.Prismatic prismatic(useFlange=true, r={1,0})
    annotation (Placement(transformation(extent={{-62,-10},{-42,-30}})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(
    Jr=0,
    VaNominal=12,
    useThermalPort=false,
    Ra=0.05,
    alpha20a(displayUnit="1/K") = 0,
    La=0.0015,
    useSupport=false,
    Js=0,
    TaOperational=293.15,
    IaNominal=1,
    wNominal(displayUnit="rad/s") = 100,
    TaNominal=293.15,
    TaRef=293.15)
    annotation (Placement(transformation(extent={{-132,20},{-112,40}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-160,20},{-140,40}})));
  Modelica.Mechanics.Translational.Components.IdealGearR2T idealGearR2T(ratio=100)
    annotation (Placement(transformation(extent={{-82,20},{-62,40}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(extent={{-22,-60},{-2,-40}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent
    annotation (Placement(transformation(extent={{-142,50},{-122,70}})));
  Modelica.Blocks.Continuous.PID PID_phi(
    k=-54.55,
    Ti=1e10,
    Td=0.0685)
            annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={90,84})));
  Modelica.Blocks.Continuous.PID PID_s(
    k=11.64,
    Ti=1e10,
    Td=1.035)
            annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={60,20})));
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{18,30},{38,10}})));
  Modelica.Blocks.Math.Add add annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,78})));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=1,
    duration=1,
    offset=0,
    startTime=7)
    annotation (Placement(transformation(extent={{-26,40},{-6,60}})));
  Tutorial2024.AuxiliaryComponents.IdealDryFriction idealDryFriction(S=6, R=2)
    annotation (Placement(transformation(extent={{-82,-10},{-62,10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=2) annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-86,78})));
  Optimization.Criteria.Signals.IntegralNorm positionError(p=1)
    annotation (Placement(transformation(extent={{78,-30},{98,-10}})));
  Optimization.Criteria.Signals.IntegralNorm angleError(p=1)
    annotation (Placement(transformation(extent={{148,-60},{168,-40}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{50,-30},{70,-10}})));
  Modelica.Blocks.Math.Product product2
    annotation (Placement(transformation(extent={{116,-60},{136,-40}})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{-12,-70},{12,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_b, rod.frame_a) annotation (Line(
      points={{-42,-60},{-42,-70},{-32,-70}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_a, carriage.frame_a) annotation (Line(
      points={{-42,-40},{-42,-20},{-22,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame, prismatic.frame_a) annotation (Line(
      points={{-92,-20},{-62,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{-42,-20},{-22,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(ground.p, dcpm.pin_an)
    annotation (Line(points={{-150,40},{-128,40}},
                                                 color={0,0,255}));
  connect(revolute1.frame_a, prismatic.frame_b) annotation (Line(
      points={{-42,-40},{-42,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(dcpm.flange, idealGearR2T.flangeR)
    annotation (Line(points={{-112,30},{-82,30}},color={0,0,0}));
  connect(idealGearR2T.flangeT, prismatic.flange_a) annotation (Line(points={{-62,30},
          {-46,30},{-46,-6},{-52,-6},{-52,-10}},
                                       color={0,127,0}));
  connect(ground.p, signalCurrent.p)
    annotation (Line(points={{-150,40},{-150,60},{-142,60}}, color={0,0,255}));
  connect(signalCurrent.n, dcpm.pin_ap)
    annotation (Line(points={{-122,60},{-116,60},{-116,40}}, color={0,0,255}));
  connect(angleSensor.flange, revolute1.flange_a)
    annotation (Line(points={{-22,-50},{-32,-50}}, color={0,0,0}));
  connect(angleSensor.phi, PID_phi.u) annotation (Line(points={{-1,-50},{108,
          -50},{108,84},{102,84}}, color={0,0,127}));
  connect(add.u2, PID_phi.y)
    annotation (Line(points={{-28,84},{79,84}}, color={0,0,127}));
  connect(add.u1, PID_s.y) annotation (Line(points={{-28,72},{80,72},{80,20},{
          71,20}}, color={0,0,127}));
  connect(idealGearR2T.flangeT, positionSensor.flange) annotation (Line(points=
          {{-62,30},{-46,30},{-46,20},{-40,20}}, color={0,127,0}));
  connect(PID_s.u, feedback.y)
    annotation (Line(points={{48,20},{37,20}}, color={0,0,127}));
  connect(positionSensor.s, feedback.u1)
    annotation (Line(points={{-19,20},{20,20}}, color={0,0,127}));
  connect(ramp.y, feedback.u2)
    annotation (Line(points={{-5,50},{28,50},{28,28}}, color={0,0,127}));
  connect(prismatic.support, idealDryFriction.flange_a) annotation (Line(points
        ={{-58,-10},{-92,-10},{-92,0},{-82,0}}, color={0,127,0}));
  connect(idealDryFriction.flange_b, prismatic.flange_a)
    annotation (Line(points={{-62,0},{-52,0},{-52,-10}}, color={0,127,0}));
  connect(add.y, limiter.u)
    annotation (Line(points={{-51,78},{-74,78}}, color={0,0,127}));
  connect(limiter.y, signalCurrent.i)
    annotation (Line(points={{-97,78},{-132,78},{-132,72}}, color={0,0,127}));
  connect(positionError.u, product1.y)
    annotation (Line(points={{76,-20},{71,-20}}, color={0,0,127}));
  connect(feedback.y, product1.u1) annotation (Line(points={{37,20},{40,20},{40,
          -14},{48,-14}}, color={0,0,127}));
  connect(feedback.y, product1.u2) annotation (Line(points={{37,20},{40,20},{40,
          -26},{48,-26}}, color={0,0,127}));
  connect(product2.u1, angleSensor.phi) annotation (Line(points={{114,-44},{108,
          -44},{108,-50},{-1,-50}}, color={0,0,127}));
  connect(angleSensor.phi, product2.u2) annotation (Line(points={{-1,-50},{108,
          -50},{108,-56},{114,-56}}, color={0,0,127}));
  connect(product2.y, angleError.u)
    annotation (Line(points={{137,-50},{146,-50}}, color={0,0,127}));
  annotation (experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-180,-100},{140,100}})),
    Icon(coordinateSystem(extent={{-180,-100},{140,100}})));
end Step11_Solution;
