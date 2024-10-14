within Tutorial2024.AuxiliaryComponents;
model CraneCrab
  extends Modelica.Blocks.Icons.Block;
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM={0,0,0},
                                               m=1) annotation (Placement(transformation(extent={{12,12},{32,32}})));
  Modelica.Mechanics.MultiBody.Parts.Body body1(r_CM={0,0,0},
                                                m=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-60})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r={1,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-26})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation (Placement(transformation(extent={{-64,12},{-44,32}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic x(useAxisFlange=true,
    s(fixed=true),
    v(fixed=true))                                                    annotation (Placement(transformation(extent={{-30,12},
            {-10,32}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange=true,
    phi(fixed=true, start=1.3962634015955),
    w(fixed=true))                                                                                                  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (Placement(transformation(extent={{30,-10},{
            50,10}})));
  Modelica.Mechanics.Translational.Sources.Force force annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-40,50})));
  Modelica.Blocks.Interfaces.RealInput f "Force" annotation (Placement(transformation(extent={{-140,-20},{-100,20}}),
        iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput phi "Angle" annotation (Placement(transformation(extent={{100,-10},{120,10}}),
        iconTransformation(extent={{100,-10},{120,10}})));
equation
  connect(fixed.frame_b, x.frame_a) annotation (Line(
      points={{-44,22},{-30,22}},
      color={95,95,95},
      thickness=0.5));
  connect(x.frame_b, body.frame_a) annotation (Line(
      points={{-10,22},{12,22}},
      color={95,95,95},
      thickness=0.5));
  connect(body.frame_a, revolute.frame_a) annotation (Line(
      points={{12,22},{0,22},{0,10}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, fixedTranslation.frame_a) annotation (Line(
      points={{0,-10},{0,-16}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedTranslation.frame_b, body1.frame_a) annotation (Line(
      points={{0,-36},{0,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(force.flange, x.axis) annotation (Line(points={{-30,50},{-12,50},{-12,28}},
                                                                             color={0,127,0}));
  connect(f, force.f) annotation (Line(points={{-120,0},{-70,0},{-70,50},{-52,50}}, color={0,0,127}));
  connect(angleSensor.phi, phi) annotation (Line(points={{51,0},{110,0}}, color={0,0,127}));
  connect(revolute.axis, angleSensor.flange) annotation (Line(points={{10,0},{30,0}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-44,-18},{46,-62}},
          lineColor={28,108,200},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{20,58},{-2,-40}},
          color={0,0,0},
          thickness=1),
        Ellipse(
          extent={{8,78},{36,48}},
          lineColor={28,108,200},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-72,-66},{78,-66}}, color={0,0,0}),
        Line(points={{-84,-86},{-56,-70}}, color={0,0,0}),
        Line(points={{-46,-86},{-18,-70}}, color={0,0,0}),
        Line(points={{-4,-86},{24,-70}}, color={0,0,0}),
        Line(points={{38,-86},{66,-70}}, color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(__Dymola_NumberOfIntervals=5000, __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>
Model of an inverse pendulum.
</p>
</html>"));
end CraneCrab;
