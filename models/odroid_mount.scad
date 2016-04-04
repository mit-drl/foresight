xlen = 80;
ylen = 57;
odroid_x = 74.44;
odroid_y = 52;
odroid_height = 25;
mount_x = 32;
mount_y = 41;
$fn = 100;
screw_d = 4;
screw_thickness = 2.5;
thickness = 3;
in_x = 65;
in_y = 25;
od_x = 60;
od_y = 40;

difference()
{
    union()
    {
        cube([xlen, ylen, thickness], true);
        for (xm = [-1, 1]) {
            for (ym = [-1, 1]) {
                translate([xm * odroid_x / 2, ym * odroid_y / 2, -1.5])
                cylinder(h=odroid_height, r=screw_d / 2 + screw_thickness);
            }
        }
    }

    for (xm = [-1, 1]) {
        for (ym = [-1, 1]) {
            translate([xm * mount_x / 2, ym * mount_y / 2, -2 * thickness])
            cylinder(h=10, r=screw_d / 2, center=false);
            translate([xm * odroid_x / 2, ym * odroid_y / 2, -10])
            cylinder(h=2 * odroid_height, r=screw_d / 2, center=false);
        }
    }
    translate([-in_x  / 2, -in_y / 2, -thickness / 2 - 1]) cube([in_x, in_y, thickness + 2]);
}
