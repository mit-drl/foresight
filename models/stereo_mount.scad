xlen = 80;
ylen = 57;
odroid_x = 74.44;
odroid_y = 52;
odroid_height = 25;
mount_x = 32;
mount_y = 39.1;
$fn = 100;
screw_d = 4;
screw_thickness = 2;
thickness = 3;
in_x = 65;
in_y = 25;
od_x = 60;
od_y = 40;

y_size = 90;
bar_r = 3;
screw_height = 8;
camera_x = 20;
camera_y = 50;
plate_x_size = 25;
plate_y_size = 55;
plate_thickness = 2;
mount_bar_r = 3;
mount_bar_y_dist = 10;
mount_bar_x_size = 20;
mount_angle = 135;

module screw_hole(x, y, z, screw_r=screw_d / 2, wall_r=screw_thickness, height=screw_height)
{
    difference()
    {
        translate([x, y, z - height / 2], center=false)
        cylinder(h=height, r=screw_r + wall_r);

        translate([x, y, z - height], center=false)
        cylinder(h=2 * height, r=screw_r, center=false);
    }
}

screw_hole(0, -y_size / 2, height=screw_height, 0);
screw_hole(0, y_size / 2, height=screw_height, 0);

translate([0, -y_size / 2 + 3])
rotate(a=[-90, 0, 0])
cylinder(h=y_size - 6, r=bar_r, center=false);

translate([0, -mount_bar_y_dist / 2, 0])
rotate(a=[0, mount_angle, 0])
cylinder(h=mount_bar_x_size, r=mount_bar_r, center=false);

translate([0, mount_bar_y_dist / 2, 0])
rotate(a=[0, mount_angle, 0])
cylinder(h=mount_bar_x_size, r=mount_bar_r, center=false);

rotate([0, 45, 0])
translate([mount_bar_x_size, -plate_y_size / 2, -plate_x_size / 2])
cube([plate_thickness, plate_y_size, plate_x_size]);

for (xm = [-1, 1])
{
    for (ym = [-1, 1])
    {
        rotate([0, 135, 0])
        screw_hole(xm * camera_x / 2,
            ym * camera_y / 2,
            mount_bar_x_size + plate_thickness / 2,
            height=plate_thickness);
    }
}

// rotate([0, 135, 0])
// translate([-camera_x / 2, 0, mount_bar_x_size + plate_thickness / 2])
// screw_hole(0, -camera_y / 2, height=plate_thickness);
