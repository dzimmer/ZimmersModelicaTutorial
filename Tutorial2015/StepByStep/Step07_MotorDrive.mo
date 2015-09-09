within Tutorial2015.StepByStep;
model Step07_MotorDrive "A motor driven crane crab"
  extends Modelica.Icons.Example;

  inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
    animateWorld=false,
    animateGravity=false)
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  BaseComponents.SimplePlanarMechanics.Parts.Body carriage(
    m=1,
    I=0,
    animate=true)
    annotation (Placement(transformation(extent={{40,-20},{60,0}})));
  BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-70,-10})));
  BaseComponents.SimplePlanarMechanics.Parts.Body pendulum(
    I=0,
    m=1,
    animate=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={80,-50})));
  BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod(r={0,0.5},
      animate=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={50,-50})));
  BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute(
    animate=true,
    w(fixed=true),
    useFlange=false,
    phi(fixed=true, start=-0.087266462599716)) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,-30})));
  BaseComponents.SimplePlanarMechanics.Joints.Prismatic prismatic(
    r={1,0},
    animate=true,
    useFlange=true,
    s(fixed=true),
    v(fixed=true))
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
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
    wNominal=104.71975511966,
    TaNominal=293.15,
    TaRef=293.15)
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  Modelica.Mechanics.Rotational.Components.IdealGearR2T idealGearR2T(ratio=100)
    annotation (Placement(transformation(extent={{-46,20},{-26,40}})));
  Modelica.Electrical.Analog.Sources.ConstantCurrent
                                                   signalCurrent(I=0.2)
    annotation (Placement(transformation(extent={{-88,70},{-68,90}})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{60,-50},{70,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, rod.frame_a) annotation (Line(
      points={{20,-40},{20,-50},{40,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_a, carriage.frame_a) annotation (Line(
      points={{20,-20},{20,-10},{40,-10}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_a, fixed.frame_a) annotation (Line(
      points={{-20,-10},{-60,-10}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{0,-10},{40,-10}},
      color={95,95,95},
      thickness=0.5));
  connect(dcpm.pin_an, ground.p) annotation (Line(
      points={{-76,40},{-90,40}},
      color={0,0,255}));
  connect(idealGearR2T.flangeR, dcpm.flange) annotation (Line(
      points={{-46,30},{-60,30}}));
  connect(ground.p, signalCurrent.p) annotation (Line(
      points={{-90,40},{-90,80},{-88,80}},
      color={0,0,255}));
  connect(signalCurrent.n, dcpm.pin_ap) annotation (Line(
      points={{-68,80},{-64,80},{-64,40}},
      color={0,0,255}));
  connect(prismatic.flange_a, idealGearR2T.flangeT) annotation (Line(
      points={{-10,-1},{-10,30},{-26,30}},
      color={0,127,0}));
  annotation (experiment(StopTime=5),
    Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end Step07_MotorDrive;
