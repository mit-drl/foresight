xlen = 30;
ylen = 32;
camera_x = 27.42;
camera_y = 28.07;
camera_height = 10;
mount_x = 14;
mount_y = 36.1;
mount_height = 10;
$fn = 100;
screw_d = 3.4;
screw_thickness = 1;
thickness = 3;
in_x = 19;
in_y = 23;

difference()
{
    union()
    {
        cube([xlen, ylen, thickness], true);
        for (xm = [-1, 1]) {
            for (ym = [-1, 1]) {
                translate([xm * camera_x / 2, ym * camera_y / 2, -1.5])
                cylinder(h=camera_height, r=screw_d / 2 + screw_thickness);
                translate([xm * mount_x / 2, ym * mount_y / 2, -mount_height + thickness / 2])
                cylinder(h=mount_height, r=screw_d / 2 + screw_thickness);
            }
        }
        translate([-mount_x / 2 - 1, mount_y / 2 - 4, -thickness / 2])
        cube([15, 6, thickness], center=false);
        translate([-mount_x / 2 -1, -mount_y / 2 - 2.5, -thickness / 2])
        cube([15, 6, thickness], center=false);
    }

    for (xm = [-1, 1]) {
        for (ym = [-1, 1]) {
            translate([xm * mount_x / 2, ym * mount_y / 2, -1.5 * mount_height])
            cylinder(h=2 * mount_height, r=screw_d / 2, center=false);
            translate([xm * camera_x / 2, ym * camera_y / 2, -10])
            cylinder(h=2 * camera_height, r=screw_d / 2, center=false);
        }
    }
    translate([-in_x  / 2, -in_y / 2, -thickness / 2 - 1]) cube([in_x, in_y, thickness + 2]);
    translate([in_x  / 2 - 1, -in_y / 2, -thickness / 2 - 1]) cube([in_x, in_y, thickness + 2]);
}
