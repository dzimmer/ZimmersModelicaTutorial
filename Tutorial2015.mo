within ;
package Tutorial2015 "Modelica modeling tutorial 2015"
  package StepByStep "Step-by-step example models"
    extends Modelica.Icons.ExamplesPackage;
    model Step01_FreeBody "A free falling body"
      extends Modelica.Icons.Example;

      inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
        animateWorld=false,
        animateGravity=false)
        annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Body freebody(
        m=1,
        animate=true,
        I=0.001,
        r(each fixed=true),
        v(each fixed=true),
        phi(fixed=true),
        w(fixed=true))
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      annotation (
 experiment(StopTime=1),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step01_FreeBody;

    model Step02_FixedBody "A body fixed on a rod"
      extends Modelica.Icons.Example;

      inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
        animateWorld=false,
        animateGravity=false)
        annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Body body(
        m=1,
        I=0,
        animate=true)
        annotation (Placement(transformation(extent={{40,0},{60,20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-30,10})));
      BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod2(animate=
            true, r={1,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={10,10})));
    equation
      connect(rod2.frame_b, body.frame_a) annotation (Line(
          points={{20,10},{40,10}},
          color={95,95,95},
          thickness=0.5));
      connect(rod2.frame_a, fixed.frame_a) annotation (Line(
          points={{0,10},{-20,10}},
          color={95,95,95},
          thickness=0.5));
      annotation (
 experiment(StopTime=1),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step02_FixedBody;

    model Step03_Pendulum "A simple pendulum"
      extends Modelica.Icons.Example;

      inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
        animateWorld=false,
        animateGravity=false)
        annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Body pendulum2(
        m=1,
        I=0,
        animate=true)
        annotation (Placement(transformation(extent={{40,0},{60,20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-70,10})));
      BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod2(animate=
            true, r={1,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={10,10})));
      BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute2(
        animate=true,
        w(fixed=true),
        useFlange=false,
        phi(fixed=true, start=0)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={-30,10})));
    equation
      connect(revolute2.frame_a, fixed.frame_a) annotation (Line(
          points={{-40,10},{-60,10}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_b, rod2.frame_a) annotation (Line(
          points={{-20,10},{0,10}},
          color={95,95,95},
          thickness=0.5));
      connect(rod2.frame_b, pendulum2.frame_a) annotation (Line(
          points={{20,10},{40,10}},
          color={95,95,95},
          thickness=0.5));
      annotation (
 experiment(StopTime=5),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step03_Pendulum;

    model Step04_DoublePendulum "A double pendulum"
      extends Modelica.Icons.Example;

      inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
        animateWorld=false,
        animateGravity=false)
        annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Body pendulum2(
        m=1,
        I=0,
        animate=true)
        annotation (Placement(transformation(extent={{40,0},{60,20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-70,10})));
      BaseComponents.SimplePlanarMechanics.Parts.Body pendulum(
        I=0,
        m=1,
        animate=true) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={80,-30})));
      BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod(r={0,0.5},
          animate=true) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={50,-30})));
      BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute(
        animate=true,
        w(fixed=true),
        useFlange=false,
        phi(fixed=true, start=0)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={20,-10})));
      BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod2(animate=
            true, r={1,0}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={-6,10})));
      BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute2(
        animate=true,
        w(fixed=true),
        useFlange=false,
        phi(fixed=true, start=0)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={-38,10})));
    equation
      connect(rod.frame_b, pendulum.frame_a) annotation (Line(
          points={{60,-30},{70,-30}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute.frame_b, rod.frame_a) annotation (Line(
          points={{20,-20},{20,-30},{40,-30}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute.frame_a, pendulum2.frame_a) annotation (Line(
          points={{20,0},{20,10},{40,10}},
          color={95,95,95},
          thickness=0.5));
      connect(rod2.frame_b, revolute.frame_a) annotation (Line(
          points={{4,10},{20,10},{20,0}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_a, fixed.frame_a) annotation (Line(
          points={{-48,10},{-60,10}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_b, rod2.frame_a) annotation (Line(
          points={{-28,10},{-16,10}},
          color={95,95,95},
          thickness=0.5));
      annotation (
 experiment(StopTime=15,
            Interval=0.01),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step04_DoublePendulum;

    model Step05_CarriageWithPendulum "A crane crab (carriage with pendulum)"
      extends Modelica.Icons.Example;

      inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
        animateWorld=false,
        animateGravity=false)
        annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Body carriage(
        m=1,
        I=0,
        animate=true)
        annotation (Placement(transformation(extent={{40,0},{60,20}})));
      BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-70,10})));
      BaseComponents.SimplePlanarMechanics.Parts.Body pendulum(
        I=0,
        m=1,
        animate=true) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={80,-30})));
      BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod(r={0,0.5},
          animate=true) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={50,-30})));
      BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute(
        animate=true,
        w(fixed=true),
        useFlange=false,
        phi(fixed=true, start=-0.087266462599716)) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={20,-10})));
      BaseComponents.SimplePlanarMechanics.Joints.Prismatic prismatic(
        r={1,0},
        animate=true,
        s(fixed=true),
        v(fixed=true),
        useFlange=false)
        annotation (Placement(transformation(extent={{-20,0},{0,20}})));
    equation
      connect(rod.frame_b, pendulum.frame_a) annotation (Line(
          points={{60,-30},{70,-30}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute.frame_b, rod.frame_a) annotation (Line(
          points={{20,-20},{20,-30},{40,-30}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute.frame_a, carriage.frame_a) annotation (Line(
          points={{20,0},{20,10},{40,10}},
          color={95,95,95},
          thickness=0.5));
      connect(prismatic.frame_a, fixed.frame_a) annotation (Line(
          points={{-20,10},{-60,10}},
          color={95,95,95},
          thickness=0.5));
      connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
          points={{0,10},{40,10}},
          color={95,95,95},
          thickness=0.5));
      annotation (
 experiment(StopTime=5),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step05_CarriageWithPendulum;

    model Step06_ActuatedCarriage "An actuated crane crab"
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
      Modelica.Mechanics.Translational.Sources.ConstantForce constantForce(
          f_constant=2)
        annotation (Placement(transformation(extent={{-50,10},{-30,30}})));
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
      connect(constantForce.flange, prismatic.flange_a) annotation (Line(
          points={{-30,20},{-10,20},{-10,-1}},
          color={0,127,0}));
      annotation (
 experiment(StopTime=5),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step06_ActuatedCarriage;

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
      annotation (
 experiment(StopTime=5),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step07_MotorDrive;

    model Step08_AngleControl "An inverse pendulum with angle control"
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
        Ra=0.05,
        alpha20a(displayUnit="1/K") = 0,
        La=0.0015,
        useSupport=false,
        Js=0,
        TaOperational=293.15,
        wNominal=104.71975511966,
        TaNominal=293.15,
        TaRef=293.15)
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
        k=-5)   annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={50,80})));
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
      connect(PID.u, angleSensor.phi) annotation (Line(
          points={{62,80},{80,80},{80,11}},
          color={0,0,127}));
      connect(prismatic.flange_a, idealGearR2T.flangeT) annotation (Line(
          points={{-10,-31},{-10,0},{-26,0}},
          color={0,127,0}));
      connect(PID.y, signalCurrent.i) annotation (Line(
          points={{39,80},{-78,80},{-78,57}},
          color={0,0,127}));
      annotation (
 experiment(StopTime=5),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step08_AngleControl;

    model Step09_PositionControl
      "An inverse pendulum with position and angle control"
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
        Ra=0.05,
        alpha20a(displayUnit="1/K") = 0,
        La=0.0015,
        useSupport=false,
        Js=0,
        TaOperational=293.15,
        wNominal=104.71975511966,
        TaNominal=293.15,
        TaRef=293.15)
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
        k=-8)   annotation (Placement(transformation(
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
        Td=0.4,
        k=2,
        yMax=1)  annotation (Placement(transformation(extent={{40,20},{60,40}})));
      Modelica.Blocks.Sources.Step step(startTime=8)
        annotation (Placement(transformation(extent={{0,24},{12,36}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder(T=0.5, initType=Modelica.Blocks.Types.Init.SteadyState)
        annotation (Placement(transformation(extent={{20,24},{32,36}})));
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
      connect(positionSensor.flange, idealGearR2T.flangeT) annotation (Line(
          points={{0,0},{-26,0}},
          color={0,127,0}));
      connect(prismatic.flange_a, idealGearR2T.flangeT) annotation (Line(
          points={{-10,-31},{-10,0},{-26,0}},
          color={0,127,0}));
      annotation (
 experiment(StopTime=15,
            Interval=0.01),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step09_PositionControl;

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
      annotation (
 experiment(StopTime=15,
            Interval=0.01),
        Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Step10_IncludingFriction;

    annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
  end StepByStep;

  package BaseComponents
    package SimplePlanarMechanics
      "A planar mechanical library for didactical purposes"
    extends Modelica.Icons.Package;
    import SI = Modelica.SIunits;
    import MB = Modelica.Mechanics.MultiBody;

      model PlanarWorld
        import SI = Modelica.SIunits;

        parameter Boolean enableAnimation=true
          "= true, if animation of all components is enabled";
        parameter Boolean animateWorld=true
          "= true, if world coordinate system shall be visualized" annotation(Dialog(enable=enableAnimation));
        parameter Boolean animateGravity=true
          "= true, if gravity field shall be visualized (acceleration vector or field center)"
                                                                                               annotation(Dialog(enable=enableAnimation));
        parameter String label1="x" "Label of horizontal axis in icon";
        parameter String label2="y" "Label of vertical axis in icon";
        parameter SI.Acceleration[2] g={0,-9.81}
          "Constant gravity acceleration vector resolved in world frame";

        parameter SI.Length axisLength=nominalLength/2
          "Length of world axes arrows"
          annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
        parameter SI.Diameter axisDiameter=axisLength/defaultFrameDiameterFraction
          "Diameter of world axes arrows"
          annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
        parameter Boolean axisShowLabels=true
          "= true, if labels shall be shown"
          annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
        input Types.Color axisColor_x=Types.Defaults.FrameColor
          "Color of x-arrow"
          annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
        input Types.Color axisColor_y=axisColor_x "Color of y-arrow"
          annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
        input Types.Color axisColor_z=axisColor_x "Color of z-arrow"
          annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));

        parameter SI.Position gravityArrowTail[2]={0,0}
          "Position vector from origin of world frame to arrow tail, resolved in world frame"
          annotation (Dialog(tab="Animation", group=
                "if animateGravity = true and gravityType = UniformGravity",
                enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
        parameter SI.Length gravityArrowLength=axisLength/2
          "Length of gravity arrow"
          annotation (Dialog(tab="Animation", group=
                "if animateGravity = true and gravityType = UniformGravity",
                enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
        parameter SI.Diameter gravityArrowDiameter=gravityArrowLength/
            defaultWidthFraction "Diameter of gravity arrow" annotation (Dialog(tab=
                "Animation", group="if animateGravity = true and gravityType = UniformGravity",
                enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
        input Types.Color gravityArrowColor={0,230,0} "Color of gravity arrow";

        parameter SI.Length defaultZPosition=0
          "Default for z positions of all the elements"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Length nominalLength=1
          "\"Nominal\" length of PlanarMechanics"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Length defaultJointLength=nominalLength/10
          "Default for the fixed length of a shape representing a joint"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Length defaultJointWidth=nominalLength/10
          "Default for the fixed width of a shape representing a joint"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Diameter defaultBodyDiameter=nominalLength/9
          "Default for diameter of sphere representing the center of mass of a body"
          annotation (Dialog(tab="Defaults"));
        parameter Real defaultWidthFraction=20
          "Default for shape width as a fraction of shape length (e.g., for Parts.FixedTranslation)"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Diameter defaultArrowDiameter=nominalLength/40
          "Default for arrow diameter (e.g., of forces, torques, sensors)"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Length defaultForceLength=nominalLength/10
          "Default for the fixed length of a shape representing a force (e.g., damper)"
          annotation (Dialog(tab="Defaults"));
        parameter SI.Length defaultForceWidth=nominalLength/20
          "Default for the fixed width of a shape represening a force (e.g., spring, bushing)"
          annotation (Dialog(tab="Defaults"));
        parameter Real defaultFrameDiameterFraction=40
          "Default for arrow diameter of a coordinate system as a fraction of axis length"
          annotation (Dialog(tab="Defaults"));
        parameter Real defaultSpecularCoefficient(min=0) = 0.7
          "Default reflection of ambient light (= 0: light is completely absorbed)"
          annotation (Dialog(tab="Defaults"));
      protected
        parameter Integer ndim=if enableAnimation and animateWorld then 1 else 0;
        parameter Integer ndim2=if enableAnimation and animateWorld and
            axisShowLabels then 1 else 0;

        // Parameters to define axes
        parameter SI.Length headLength=min(axisLength, axisDiameter*Types.Defaults.
            FrameHeadLengthFraction);
        parameter SI.Length headWidth=axisDiameter*Types.Defaults.
            FrameHeadWidthFraction;
        parameter SI.Length lineLength=max(0, axisLength - headLength);
        parameter SI.Length lineWidth=axisDiameter;

        // Parameters to define axes labels
        parameter SI.Length scaledLabel=Modelica.Mechanics.MultiBody.Types.Defaults.FrameLabelHeightFraction*
            axisDiameter;
        parameter SI.Length labelStart=1.05*axisLength;

        // x-axis
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowLine(
          shapeType="cylinder",
          length=lineLength,
          width=lineWidth,
          height=lineWidth,
          lengthDirection={1,0,0},
          widthDirection={0,1,0},
          color=axisColor_x,
          specularCoefficient=0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowHead(
          shapeType="cone",
          length=headLength,
          width=headWidth,
          height=headWidth,
          lengthDirection={1,0,0},
          widthDirection={0,1,0},
          color=axisColor_x,
          r={lineLength,0,0},
          specularCoefficient=0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines x_label(
          lines=scaledLabel*{[0, 0; 1, 1],[0, 1; 1, 0]},
          diameter=axisDiameter,
          color=axisColor_x,
          r_lines={labelStart,0,0},
          n_x={1,0,0},
          n_y={0,1,0},
          specularCoefficient=0) if enableAnimation and animateWorld and axisShowLabels;

        // y-axis
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowLine(
          shapeType="cylinder",
          length=lineLength,
          width=lineWidth,
          height=lineWidth,
          lengthDirection={0,1,0},
          widthDirection={1,0,0},
          color=axisColor_y,
          specularCoefficient=0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowHead(
          shapeType="cone",
          length=headLength,
          width=headWidth,
          height=headWidth,
          lengthDirection={0,1,0},
          widthDirection={1,0,0},
          color=axisColor_y,
          r={0,lineLength,0},
          specularCoefficient=0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines y_label(
          lines=scaledLabel*{[0, 0; 1, 1.5],[0, 1.5; 0.5, 0.75]},
          diameter=axisDiameter,
          color=axisColor_y,
          r_lines={0,labelStart,0},
          n_x={0,1,0},
          n_y={-1,0,0},
          specularCoefficient=0) if enableAnimation and animateWorld and axisShowLabels;

        // z-axis
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowLine(
          shapeType="cylinder",
          length=lineLength,
          width=lineWidth,
          height=lineWidth,
          lengthDirection={0,0,1},
          widthDirection={0,1,0},
          color=axisColor_z,
          specularCoefficient=0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowHead(
          shapeType="cone",
          length=headLength,
          width=headWidth,
          height=headWidth,
          lengthDirection={0,0,1},
          widthDirection={0,1,0},
          color=axisColor_z,
          r={0,0,lineLength},
          specularCoefficient=0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines z_label(
          lines=scaledLabel*{[0, 0; 1, 0],[0, 1; 1, 1],[0, 1; 1, 0]},
          diameter=axisDiameter,
          color=axisColor_z,
          r_lines={0,0,labelStart},
          n_x={0,0,1},
          n_y={0,1,0},
          specularCoefficient=0) if enableAnimation and animateWorld and axisShowLabels;

        // gravity visualization
        parameter SI.Length gravityHeadLength=min(gravityArrowLength,
            gravityArrowDiameter*Types.Defaults.ArrowHeadLengthFraction);
        parameter SI.Length gravityHeadWidth=gravityArrowDiameter*Types.Defaults.ArrowHeadWidthFraction;
        parameter SI.Length gravityLineLength=max(0, gravityArrowLength - gravityHeadLength);
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowLine(
          shapeType="cylinder",
          length=gravityLineLength,
          width=gravityArrowDiameter,
          height=gravityArrowDiameter,
          lengthDirection={g[1],g[2],0},
          widthDirection={0,1,0},
          color=gravityArrowColor,
          r_shape={gravityArrowTail[1],gravityArrowTail[2],0},
          specularCoefficient=0) if enableAnimation and animateGravity;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowHead(
          shapeType="cone",
          length=gravityHeadLength,
          width=gravityHeadWidth,
          height=gravityHeadWidth,
          lengthDirection={g[1],g[2],0},
          widthDirection={0,1,0},
          color=gravityArrowColor,
          r_shape={gravityArrowTail[1],gravityArrowTail[2],0} + Modelica.Math.Vectors.normalize({g[1],g[2],0})*gravityLineLength,
          specularCoefficient=0) if enableAnimation and animateGravity;
          annotation (
          defaultComponentName="planarWorld",
          defaultComponentPrefixes="inner",
          missingInnerMessage="No \"world\" component is defined. A default world
component with the default gravity field will be used
(g=9.81 in negative y-axis). If this is not desired,
drag PlanarMechanics.PlanarWorld into the top level of your model.",
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-100,-118},{-100,61}},
                thickness=0.5),
              Polygon(
                points={{-100,100},{-120,60},{-80,60},{-100,100},{-100,100}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-119,-100},{59,-100}},
                thickness=0.5),
              Polygon(
                points={{99,-100},{59,-80},{59,-120},{99,-100}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-150,145},{150,105}},
                textString="%name",
                lineColor={0,0,255}),
              Text(
                extent={{95,-113},{144,-162}},
                lineColor={0,0,0},
                textString="%label1"),
              Text(
                extent={{-170,127},{-119,77}},
                lineColor={0,0,0},
                textString="%label2"),
              Line(points={{-56,78},{-56,-26}}, color={0,0,255}),
              Polygon(
                points={{-68,-26},{-56,-66},{-44,-26},{-68,-26}},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,255}),
              Line(points={{2,78},{2,-26}}, color={0,0,255}),
              Polygon(
                points={{-10,-26},{2,-66},{14,-26},{-10,-26}},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,255}),
              Line(points={{66,80},{66,-26}}, color={0,0,255}),
              Polygon(
                points={{54,-26},{66,-66},{78,-26},{54,-26}},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,255})}),
          Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Model <b>PlanarWorld</b> defines all possible general parameters to make parameterization of models much more convenient. It has the following functionalites.</p>
<p><ol>
<li>It defines the global coordinate system fixed in ground and shows the x, y, z axes in animation if wanted. </li>
<li>It contains all default parameters for animation, e.g. axis diameter, default joint length etc, which can still be overwritten by setting parameters in these models.</li>
<li>It provides the default gravity definition and its animation.</li>
</ol></p>
</html>"));
      end PlanarWorld;


      package Interfaces
        extends Modelica.Icons.InterfacesPackage;

        connector Frame "General Connector for planar mechanical components"
          SI.Position x "x-position";
          SI.Position y "y-position";
          SI.Angle phi "angle (counter-clockwise)";
          flow SI.Force fx "force in x-direction";
          flow SI.Force fy "force in y-direction";
          flow SI.Torque t "torque (clockwise)";
          annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Frame is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system. Normally, this connector is fixed to a mechanical component. But this model is never used directly in a system. It is only for usage of inheritance.</p>
</html>"));
        end Frame;

        connector Frame_a
          extends Frame;
          annotation (Icon(graphics={
                Rectangle(
                  extent={{-40,100},{40,-100}},
                  lineColor={95,95,95},
                  fillColor={203,237,255},
                  fillPattern=FillPattern.Solid,
                  lineThickness=0.5),
                Line(
                  points={{-18,0},{22,0}},
                  color={95,95,95}),
                Line(
                  points={{0,20},{0,-20}},
                  color={95,95,95})}), Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Frame_a is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system. Normally, this connector is fixed to a mechanical component. The same as <a href=\"modelica://PlanarMechanics.Interfaces.Frame_b\">Frame_b</a>.</p>
</html>"));
        end Frame_a;

        connector Frame_b
          extends Frame;
          annotation (Icon(graphics={
                Rectangle(
                  extent={{-40,100},{40,-100}},
                  lineColor={95,95,95},
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineThickness=0.5),
                Line(
                  points={{-18,0},{22,0}},
                  color={95,95,95}),
                Line(
                  points={{0,20},{0,-20}},
                  color={95,95,95})}), Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Frame_b is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system. Normally, this connector is fixed to a mechanical component. The same as <a href=\"modelica://PlanarMechanics.Interfaces.Frame_a\">Frame_a</a>.</p>
</html>"));
        end Frame_b;

        partial model PartialTwoFlanges "Partial model with 2 flanges"

          Frame_a frame_a
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-120,-20},{-80,20}})));
          Frame_b frame_b
            annotation (Placement(transformation(extent={{90,-10},{110,10}}),
                iconTransformation(extent={{80,-20},{120,20}})));
          annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>This is a partial model with 2 planar flanges. it can be inherited to build up models with 2 planar flanges.</p>
</html>"));
        end PartialTwoFlanges;
      annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>This package contains connectors and partial models (i.e., models that are only used to build other models) of the PlanarMechanics library. </p>
</html>"));
      end Interfaces;

      package Parts
        model Body "Body component with mass and inertia"

          Interfaces.Frame_a frame_a
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-120,-20},{-80,20}})));
          outer PlanarWorld planarWorld "planar world model";
          parameter Boolean animate = true
            "= true, if animation shall be enabled";
          parameter StateSelect stateSelect=StateSelect.default
            "Priority to use phi, w and a as states" annotation(HideResult=true,Dialog(tab="Advanced"));
          parameter SI.Mass m "mass of the body";
          parameter SI.Inertia I "Inertia of the Body";
          parameter SI.Acceleration g[2] = planarWorld.g
            "local gravity acting on the mass";
          parameter SI.Length zPosition = planarWorld.defaultZPosition
            "z position of the body" annotation (Dialog(
              tab="Animation",
              group="if animation = true",
              enable=animate));
          parameter SI.Diameter sphereDiameter=planarWorld.defaultBodyDiameter
            "Diameter of sphere" annotation (Dialog(
              tab="Animation",
              group="if animation = true",
              enable=animate));
          input Modelica.Mechanics.MultiBody.Types.SpecularCoefficient
            specularCoefficient = planarWorld.defaultSpecularCoefficient
            "Reflection of ambient light (= 0: light is completely absorbed)"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          SI.Force f[2] "force";
          SI.Position r[2](each final stateSelect=stateSelect, start={0,0})
            "transl. position"                                                            annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.Velocity v[2](each final stateSelect=stateSelect, start={0,0})
            "velocity"                                                            annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.Acceleration a[2](start={0,0}) "acceleration" annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.Angle phi(final stateSelect=stateSelect, start=0) "angle" annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.AngularVelocity w(final stateSelect=stateSelect, start = 0)
            "angular velocity"                                                              annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.AngularAcceleration z(start = 0) "angular acceleration"
                                   annotation(Dialog(group="Initialization", showStartAttribute=true));
          //Visualization
          MB.Visualizers.Advanced.Shape sphere(
            shapeType="sphere",
            color={63,63,255},
            specularCoefficient=specularCoefficient,
            length=sphereDiameter,
            width=sphereDiameter,
            height=sphereDiameter,
            lengthDirection={0,0,1},
            widthDirection={1,0,0},
            r_shape={0,0,0} -{0,0,1}*sphereDiameter/2,
            r={frame_a.x,frame_a.y,zPosition},
            R=MB.Frames.axisRotation(3,frame_a.phi,w)) if  planarWorld.enableAnimation and animate;
        equation
          //The velocity is a time-derivative of the position
          r = {frame_a.x, frame_a.y};
          v = der(r);
          phi = frame_a.phi;
          w = der(frame_a.phi);
          //The acceleration is a time-derivative of the velocity
          a = der(v);
          z = der(w);
          //Newton's law
          f = {frame_a.fx, frame_a.fy};
          f + m*g = m*a;
          frame_a.t = I*z;
          annotation (Icon(graphics={
                Rectangle(
                  extent={{-100,40},{-20,-40}},
                  lineColor={0,0,0},
                  fillColor={85,170,255},
                  fillPattern=FillPattern.HorizontalCylinder),
                Ellipse(
                  extent={{-60,60},{60,-60}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255}),
                Text(
                  extent={{-100,-80},{100,-120}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255},
                  textString="%name")}),            Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Model <b>Body</b> is a ideal unlimited small point with mass and inertia. </p>
</html>"));
        end Body;

        model Fixed "FixedPosition"
          Interfaces.Frame_a frame_a
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-120,-20},{-80,20}})));
          parameter SI.Position r[2] = {0,0} "fixed x,y-position";
          parameter SI.Angle phi = 0 "fixed angle";
        equation
          {frame_a.x,frame_a.y} = r;
          frame_a.phi = phi;
          annotation (Icon(graphics={
                Text(
                  extent={{-100,-80},{100,-120}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255},
                  textString="%name"),
                Line(
                  points={{-92,0},{0,0}}),
                Line(
                  points={{0,80},{0,-80}}),
                Line(
                  points={{0,40},{80,0}}),
                Line(
                  points={{0,80},{80,40}}),
                Line(
                  points={{0,0},{80,-40}}),
                Line(
                  points={{0,-40},{80,-80}})}),            Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>This component defines the x, y-position and angle of the frame connectors, to which this component is attached to.</p>
</html>"));
        end Fixed;

        model FixedRotation
          "A fixed translation between two components (rigid rod)"
          extends Interfaces.PartialTwoFlanges;
          outer PlanarWorld planarWorld "planar world model";
          parameter SI.Angle alpha "fixed rotation angle";
          parameter Boolean animate = true
            "= true, if animation shall be enabled"
                                                   annotation(Dialog(group="Animation"));
          parameter SI.Length zPosition = planarWorld.defaultZPosition
            "z position of z position of cylinder representing the fixed rotation"
                                                                                   annotation (Dialog(
              tab="Animation",
              group="if animation = true",
              enable=animate));
          parameter SI.Length cylinderLength=planarWorld.defaultJointLength
            "Length of cylinder representing the fixed rotation"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          parameter SI.Length cylinderDiameter=planarWorld.defaultJointWidth
            "Diameter of cylinder representing the fixed rotation"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          input Modelica.Mechanics.MultiBody.Types.Color cylinderColor={155,155,155}
            "Color of cylinder representing the fixed rotation"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          input Modelica.Mechanics.MultiBody.Types.SpecularCoefficient
            specularCoefficient = planarWorld.defaultSpecularCoefficient
            "Reflection of ambient light (= 0: light is completely absorbed)"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          //Visualization
          MB.Visualizers.Advanced.Shape cylinder(
            shapeType="cylinder",
            color=cylinderColor,
            specularCoefficient=specularCoefficient,
            length=cylinderLength,
            width=cylinderDiameter,
            height=cylinderDiameter,
            lengthDirection={0,0,1},
            widthDirection={1,0,0},
            r_shape={0,0,-0.05},
            r={frame_a.x,frame_a.y,zPosition},
            R=MB.Frames.nullRotation()) if planarWorld.enableAnimation and animate;
        equation
          frame_a.x = frame_b.x;
          frame_a.y = frame_b.y;
          frame_a.phi + alpha = frame_b.phi;
          frame_a.fx + frame_b.fx = 0;
          frame_a.fy + frame_b.fy = 0;
          frame_a.t + frame_b.t = 0;
          annotation (Icon(graphics={
                Text(
                  extent={{-100,-40},{100,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255},
                  textString="%name"),
                Polygon(
                  points={{4,48},{92,8},{92,-12},{0,32},{-92,-10},{-92,8},{-6,48},{4,48}},
                  lineColor={0,0,0},
                  fillColor={175,175,175},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-20,60},{20,20}},
                  lineColor={0,0,0},
                  fillColor={175,175,175},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-10,50},{10,30}},
                  lineColor={255,255,255},
                  fillColor={175,175,175},
                  fillPattern=FillPattern.Solid,
                  lineThickness=0.5)}),            Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>This component assures a static angle difference <b>alpha</b> between two frame connectors, to which <b>frame_a</b> and <b>frame_b</b> are connected.</p>
</html>"));
        end FixedRotation;

        model FixedTranslation
          "A fixed translation between two components (rigid rod)"

          Interfaces.Frame_a frame_a
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-120,-20},{-80,20}})));
          Interfaces.Frame_b frame_b annotation (Placement(transformation(extent={{90,-10},
                    {110,10}}), iconTransformation(extent={{80,-20},{120,20}})));
          outer PlanarWorld planarWorld "planar world model";
          parameter SI.Length r[2] = {1,0}
            "length of the rod resolved w.r.t to body frame at phi = 0";
          final parameter SI.Length l = sqrt(r*r);
          SI.Position r0[2] "length of the rod resolved w.r.t to inertal frame";
          Real R[2,2] "Rotation matrix";
          parameter Boolean animate = true
            "= true, if animation shall be enabled"
                                                   annotation(Dialog(group="Animation"));
          parameter SI.Length zPosition = planarWorld.defaultZPosition
            "z position of cylinder representing the fixed translation" annotation (Dialog(
              tab="Animation", group="if animation = true", enable=animate));
          parameter SI.Distance width=l/planarWorld.defaultWidthFraction
            "Width of shape"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animation));
          input Modelica.Mechanics.MultiBody.Types.SpecularCoefficient
            specularCoefficient = planarWorld.defaultSpecularCoefficient
            "Reflection of ambient light (= 0: light is completely absorbed)"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animation));
          //Visualization
          MB.Visualizers.Advanced.Shape cylinder(
            shapeType="cylinder",
            color={128,128,128},
            specularCoefficient=specularCoefficient,
            length=l,
            width=width,
            height=width,
            lengthDirection={r0[1]/l,r0[2]/l,0},
            widthDirection={0,0,1},
            r_shape={0,0,0},
            r={frame_a.x,frame_a.y,zPosition},
            R=MB.Frames.nullRotation()) if planarWorld.enableAnimation and animate;
        equation
          //resolve the rod w.r.t inertial system
        //  sx0 = cos(frame_a.phi)*sx + sin(frame_a.phi)*sy;
        //  sy0 = -sin(frame_a.phi)*sx + cos(frame_a.phi)*sy;
          R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi),cos(frame_a.phi)}};
          r0 = R*r;
          //rigidly connect positions
          frame_a.x + r0[1] = frame_b.x;
          frame_a.y + r0[2] = frame_b.y;
          frame_a.phi = frame_b.phi;
          //balance forces including lever principle
          frame_a.fx + frame_b.fx = 0;
          frame_a.fy + frame_b.fy = 0;
        //  frame_a.t + frame_b.t - sx0*frame_b.fy + sy0*frame_b.fx = 0;
          frame_a.t  + frame_b.t + r0*{frame_b.fy,-frame_b.fx} = 0;
          annotation (Icon(graphics={
                Text(
                  extent={{-100,-40},{100,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255},
                  textString="%name"), Rectangle(
                  extent={{-92,6},{92,-6}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Solid,
                  fillColor={175,175,175})}),            Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>This component assures a static position difference <b>r</b> between two frame connectors, to which <b>frame_a</b> and <b>frame_b</b> are connected.</p>
</html>"));
        end FixedTranslation;
      annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Package <b>Parts</b> contains rigid components and spring/damper components for planar mechanical systems, which could be used to build up a complex planar system.</p>
</html>"));
      end Parts;

      package Joints "Planar joint models"
        model Prismatic "A prismatic joint"

          Interfaces.Frame_a frame_a annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
                    iconTransformation(extent={{-120,-20},{-80,20}})));
          Interfaces.Frame_b frame_b annotation (Placement(transformation(extent={{90,-10},
                    {110,10}}), iconTransformation(extent={{80,-20},{120,20}})));

          outer PlanarWorld planarWorld "planar world model";
          parameter Boolean useFlange=false
            "= true, if force flange enabled, otherwise implicitly grounded"
              annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
          parameter StateSelect stateSelect=StateSelect.default
            "Priority to use acceleration as states" annotation(HideResult=true,Dialog(tab="Advanced"));

          parameter SI.Position r[2]
            "direction of the rod wrt. body system at phi=0";
          final parameter SI.Length l = sqrt(r*r) "length of r";
          final parameter SI.Distance e[2]= r/l "normalized r";
          SI.Position s(final stateSelect = stateSelect, start = 0)
            "Elongation of the joint";
          Real e0[2]
            "direction of the prismatic rod resolved wrt.inertial frame";
          SI.Position r0[2]
            "translation vector of the prismatic rod resolved wrt.inertial frame";
          Real R[2,2] "Rotation Matrix";
          SI.Velocity v(final stateSelect = stateSelect, start = 0)
            "velocity of elongation";
          SI.Acceleration a(start = 0) "acceleration of elongation"                          annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.Force f "force in direction of elongation";

          Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a(f = f, s = s) if useFlange
           annotation (
              Placement(transformation(extent={{-8,80},{12,100}}), iconTransformation(
                  extent={{-10,80},{10,100}})));

          parameter Boolean animate = true
            "= true, if animation shall be enabled"
                                                   annotation(Dialog(group="Animation"));

          parameter SI.Length zPosition = planarWorld.defaultZPosition
            "z position of the prismatic joint box" annotation (Dialog(
              tab="Animation",
              group="if animation = true",
              enable=animate));
          parameter SI.Distance boxWidth=l/planarWorld.defaultWidthFraction
            "Width of prismatic joint box"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animation));
          input Types.Color boxColor=Types.Defaults.JointColor
            "Color of prismatic joint box"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animation));
          input Modelica.Mechanics.MultiBody.Types.SpecularCoefficient
            specularCoefficient = planarWorld.defaultSpecularCoefficient
            "Reflection of ambient light (= 0: light is completely absorbed)"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          //Visualization
          MB.Visualizers.Advanced.Shape box(
            shapeType="box",
            color=boxColor,
            specularCoefficient=specularCoefficient,
            length=s,
            width=boxWidth,
            height=boxWidth,
            lengthDirection={e0[1],e0[2],0},
            widthDirection={0,0,1},
            r_shape={0,0,0},
            r={frame_a.x,frame_a.y,zPosition},
            R=MB.Frames.nullRotation()) if planarWorld.enableAnimation and animate;

        equation
          //resolve the rod w.r.t. inertial system
          R = {{cos(frame_a.phi), -sin(frame_a.phi)}, {sin(frame_a.phi),cos(frame_a.phi)}};
          e0 = R*e;
          r0 = e0*s;
          //differential equations
          v = der(s);
          a = der(v);
          //actuation force
          if not useFlange then
            f = 0;
          end if;
          //rigidly connect positions
          frame_a.x + r0[1] = frame_b.x;
          frame_a.y + r0[2] = frame_b.y;
          frame_a.phi = frame_b.phi;
          //balance forces including lever principle
          frame_a.fx + frame_b.fx = 0;
          frame_a.fy + frame_b.fy = 0;
          frame_a.t  + frame_b.t + r0*{frame_b.fy,-frame_b.fx} = 0;
          {frame_a.fx,frame_a.fy}*e0 = f;
          annotation (Icon(graphics={
                Text(
                  extent={{-100,-60},{100,-100}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255},
                  textString="%name"),
                Rectangle(
                  extent={{-100,40},{-20,-40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={175,175,175}),
                Rectangle(
                  extent={{-20,-20},{100,20}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={175,175,175}),
                Line(
                  visible=useFlange,
                  points={{0,80},{0,20}})}),            Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Direction of the Joint is determined by <b>r[2]</b>, which is a vector pointing from <b>frame_a</b> to <b>frame_b</b>. </p>
<p>By setting <b>useFlange</b> as true, the flange for a force input will be activated. In the &quot;Initialization&quot; block, elongation of the joint <b>s</b>, velocity of elongation <b>v</b> as well as acceleration of elongation <b>a</b> can be initialized.</p>
<p>It can be defined via parameter (in &quot;advanced&quot; tab) <b>stateSelect</b> that the relative distance &quot;s&quot; and its derivative shall be definitely used as states by setting stateSelect=StateSelect.always. </p>
<p>In &quot;Animation&quot; Tab, animation parameters for this model can be set, where <b>zPosition</b> represents the model&apos;s position along the z axis in 3D animation. Some of the values can be preset by a outer PlanarWorld model.</p>
</html>"));
        end Prismatic;

        model Revolute "A revolute joint"

          Interfaces.Frame_a frame_a
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-120,-20},{-80,20}})));
          Interfaces.Frame_b frame_b annotation (Placement(transformation(extent={{90,-10},
                    {110,10}}), iconTransformation(extent={{80,-20},{120,20}})));
          outer PlanarWorld planarWorld "planar world model";
          parameter Boolean useFlange=false
            "= true, if force flange enabled, otherwise implicitly grounded"
              annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
          //parameter Boolean initialize = false "Initialize Position and Velocity";
          parameter StateSelect stateSelect=StateSelect.default
            "Priority to use phi and w as states" annotation(HideResult=true,Dialog(tab="Advanced"));

          Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a(phi = phi, tau = t) if useFlange annotation (
              Placement(transformation(extent={{-10,90},{10,110}}),iconTransformation(
                  extent={{-10,90},{10,110}})));
          parameter Boolean animate = true
            "= true, if animation shall be enabled"
                                                   annotation(Dialog(group="Animation"));

          parameter SI.Length zPosition = planarWorld.defaultZPosition
            "z position of cylinder representing the joint axis" annotation (Dialog(
              tab="Animation",
              group="if animation = true",
              enable=animate));
          parameter SI.Distance cylinderLength=planarWorld.defaultJointLength
            "Length of cylinder representing the joint axis"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          parameter SI.Distance cylinderDiameter=planarWorld.defaultJointWidth
            "Diameter of cylinder representing the joint axis"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          input Modelica.Mechanics.MultiBody.Types.Color cylinderColor=Types.Defaults.JointColor
            "Color of cylinder representing the joint axis"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          input Modelica.Mechanics.MultiBody.Types.SpecularCoefficient
            specularCoefficient = planarWorld.defaultSpecularCoefficient
            "Reflection of ambient light (= 0: light is completely absorbed)"
            annotation (Dialog(tab="Animation", group="if animation = true", enable=animate));
          SI.Angle phi(final stateSelect = stateSelect, start = 0)
            "Angular position" annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.AngularVelocity w(final stateSelect = stateSelect, start = 0)
            "Angular velocity"
                              annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.AngularAcceleration z(start = 0) "Angular acceleration"                          annotation(Dialog(group="Initialization", showStartAttribute=true));
          SI.Torque t "Torque";
          //Visualization
          MB.Visualizers.Advanced.Shape cylinder(
            shapeType="cylinder",
            color=cylinderColor,
            specularCoefficient=specularCoefficient,
            length=cylinderLength,
            width=cylinderDiameter,
            height=cylinderDiameter,
            lengthDirection={0,0,1},
            widthDirection={1,0,0},
            r_shape={0,0,-0.05},
            r={frame_a.x,frame_a.y,zPosition},
            R=MB.Frames.nullRotation()) if planarWorld.enableAnimation and animate;

        equation
          //Differential Equations
          w = der(phi);
          z = der(w);
          //actutation torque
          if not useFlange then
            t = 0;
          end if;
          //rigidly connect positions
          frame_a.x = frame_b.x;
          frame_a.y = frame_b.y;
          frame_a.phi + phi = frame_b.phi;
          //balance forces
          frame_a.fx + frame_b.fx = 0;
          frame_a.fy + frame_b.fy = 0;
          frame_a.t + frame_b.t = 0;
          frame_a.t = t;
          annotation (Icon(graphics={
                Text(
                  extent={{-100,-80},{100,-120}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.Sphere,
                  fillColor={85,170,255},
                  textString="%name"), Rectangle(
                  extent={{-20,20},{20,-20}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={175,175,175}),
                                       Rectangle(
                  extent={{-100,60},{-20,-62}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={175,175,175}),
                                       Rectangle(
                  extent={{20,60},{100,-60}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={175,175,175}),
                Line(
                  visible=useFlange,
                  points={{0,80},{0,20}})}),            Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>Joint where frame_b rotates around axis n which is fixed in frame_a. The two frames coincide when the rotation angle &quot;phi = 0&quot;. </p>
<p>By setting <b>useFlange</b> as true, the flange for a torque input will be activated. In the &quot;Initialization&quot; block, angular position <b>phi</b>, angular velocity <b>w</b> as well as angular acceleration <b>z</b> can be initialized.</p>
<p>It can be defined via parameter (in &quot;advanced&quot; tab) <b>stateSelect</b> that the relative distance &quot;s&quot; and its derivative shall be definitely used as states by setting stateSelect=StateSelect.always. </p>
<p>In &quot;Animation&quot; Tab, animation parameters for this model can be set, where <b>zPosition</b> represents the model&apos;s position along the z axis in 3D animation. Some of the values can be preset by a outer PlanarWorld model.</p>
</html>"));
        end Revolute;
      annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>This package contains idealized, massless <b>joint components </b>and<b> Rolling components</b>.</p>
</html>"));
      end Joints;

      package Types
        type Color = Modelica.Icons.TypeInteger[3] (each min=0, each max=255)
          "RGB representation of color (will be improved with a color editor)"
          annotation (
            Dialog(colorSelector),
            choices(
              choice={0,0,0} "{0,0,0}       \"black\"",
              choice={155,0,0} "{155,0,0}     \"dark red\"",
              choice={255,0,0} "{255,0,0 }    \"red\"",
              choice={255,65,65} "{255,65,65}   \"light red\"",
              choice={0,128,0} "{0,128,0}     \"dark green\"",
              choice={0,180,0} "{0,180,0}     \"green\"",
              choice={0,230,0} "{0,230,0}     \"light green\"",
              choice={0,0,200} "{0,0,200}     \"dark blue\"",
              choice={0,0,255} "{0,0,255}     \"blue\"",
              choice={0,128,255} "{0,128,255}   \"light blue\"",
              choice={255,255,0} "{255,255,0}   \"yellow\"",
              choice={255,0,255} "{255,0,255}   \"pink\"",
              choice={100,100,100} "{100,100,100} \"dark grey\"",
              choice={155,155,155} "{155,155,155} \"grey\"",
              choice={255,255,255} "{255,255,255} \"white\""),
          Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>
Type <b>Color</b> is an Integer vector with 3 elements,
{r, g, b}, and specifies the color of a shape.
{r,g,b} are the \"red\", \"green\" and \"blue\" color parts.
Note, r g, b are given in the range 0 .. 255.
</p>
</html>"));
        package Defaults
          "Default settings of the MultiBody library via constants"
          extends Modelica.Icons.Package;

          // Color defaults
          constant Types.Color BodyColor={0,128,255}
            "Default color for body shapes that have mass (light blue)";
          constant Types.Color RodColor={155,155,155}
            "Default color for massless rod shapes (grey)";
          constant Types.Color JointColor={255,0,0}
            "Default color for elementary joints (red)";
          constant Types.Color ForceColor={0,128,0}
            "Default color for force arrow (dark green)";
          constant Types.Color TorqueColor={0,128,0}
            "Default color for torque arrow (dark green)";
          constant Types.Color SpringColor={0,0,255}
            "Default color for a spring (blue)";
          constant Types.Color SensorColor={255,255,0}
            "Default color for sensors (yellow)";
          constant Types.Color FrameColor={0,0,255}
            "Default color for frame axes and labels (blue)";
          constant Types.Color ArrowColor={0,0,255}
            "Default color for arrows and double arrows (blue)";

          // Arrow and frame defaults
          constant Real FrameHeadLengthFraction=5.0
            "Frame arrow head length / arrow diameter";
          constant Real FrameHeadWidthFraction=3.0
            "Frame arrow head width / arrow diameter";
          constant Real FrameLabelHeightFraction=3.0
            "Height of frame label / arrow diameter";
          constant Real ArrowHeadLengthFraction=4.0
            "Arrow head length / arrow diameter";
          constant Real ArrowHeadWidthFraction=3.0
            "Arrow head width / arrow diameter";

          // Other defaults
          constant SI.Diameter BodyCylinderDiameterFraction=3
            "Default for body cylinder diameter as a fraction of body sphere diameter";
          constant Real JointRodDiameterFraction=2
            "Default for rod diameter as a fraction of joint sphere diameter attached to rod";

          /*
  constant Real N_to_m(unit="N/m") = 1000
    "Default force arrow scaling (length = force/N_to_m_default)";
  constant Real Nm_to_m(unit="N.m/m") = 1000
    "Default torque arrow scaling (length = torque/Nm_to_m_default)";
*/

          annotation ( Documentation(revisions="<html> <p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>
This package contains constants used as default setting
in the PlanarMechanics library.
</p>
</html>"));
        end Defaults;
      annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",  info="<html>
<p>In this package <b>types</b> and <b>constants</b> are defined that are used in the PlanarMechanics library. The types have additional annotation choices definitions that define the menus to be built up in the graphical user interface when the type is used as parameter in a declaration. </p>
</html>"));
      end Types;
      annotation (
      preferredView="info", Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",     info="<html>
<p>Library <b>PlanarMechanics</b> is a <b>free</b> Modelica package providing 2-dimensional mechanical components to model mechanical systems, such as robots, mechanisms, vehicles, where MultiBody library is unnecessarily complex.</p>
<p>This variant is a heavily reduced and simplified variant of the free planar Mechanics library <a href=\"https://github.com/DLR-SR/PlanarMechanics\"> on  GitHub</a> </p>
</html>"
),
        Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
         graphics={
        Ellipse(
          extent={{-46,10},{-26,-8}},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-74,-42},{-54,-60}},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{18,-34},{38,-52}},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-64,-50},{-36,0},{24,-42}})}),
      Diagram(coordinateSystem(extent={{-120,-100},{80,100}})));
    end SimplePlanarMechanics;

    package Friction "Friction models"
      model IdealDryFriction "Ideal dry (Coulomb) friction"
        extends Modelica.Mechanics.Translational.Interfaces.PartialCompliant;
        extends Modelica.Mechanics.Translational.Interfaces.PartialFriction(final v_small=1e-3);
        parameter Modelica.SIunits.Force S = 10 "Sticking friction force";
        parameter Modelica.SIunits.Force R = 8 "Sliding friction force";

      equation
        free = false;
        f0 = R;
        f0_max = S;

        // velocity and acceleration of flanges
        v_relfric = der(-s_rel);
        a_relfric = der(v_relfric);

        // Friction force
        -f = if locked then sa*unitForce else (
           if startForward then R else if startBackward then -R else if pre(mode) == Forward then R else -R
         );

        annotation (                               Icon(graphics={
              Rectangle(
                extent={{-42,40},{58,12}},
                pattern=LinePattern.None,
                fillColor={175,175,175},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-62,-12},{38,-40}},
                pattern=LinePattern.None,
                fillColor={175,175,175},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-92,0},{-62,0},{-62,-40},{38,-40},{38,-12}},
                pattern=LinePattern.None),
              Line(
                points={{88,0},{58,0},{58,40},{-42,40},{-42,12}},
                pattern=LinePattern.None),
              Text(
                extent={{100,100},{-100,60}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
      end IdealDryFriction;
      annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
    end Friction;

    annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
  end BaseComponents;
  annotation (uses(Modelica(version="3.2.1")),
      Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
<p>Acknowledged Contributors: Thomas Beutlich, ITI GmbH for various changes and fixes helping the library and many of its components to work on many tools</p>
</html>"));
end Tutorial2015;
