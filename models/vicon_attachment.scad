difference()
{
square([30, 18], true);
translate([10, 0, 0])
circle(d=3.8, $fn=50, center=false);
translate([14.1, -9.1, 0])
polygon(points=[[-13,0], [1, 0], [1, 7]]);
translate([14.1, 9.1, 0])
polygon(points=[[-13,0], [1, 0], [1, -7]]);	
}