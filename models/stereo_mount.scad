
$fn = 100;
screw_thickness = 2;
mount_screw_d = 4;
camera_screw_d = 3;
y_size = 154 + 28;
bar_r = 3;
screw_height = 8;
camera_x = 12.83 - 1.55;
camera_y = 72.86 - 1.55;
plate_x_size = 8;
plate_y_size = 75;
plate_thickness = 3;
mount_bar_r = 3;
mount_bar_y_dist = 30;
mount_bar_x_size = 20;
mount_angle = 135;
mount_ext = 10;

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

screw_hole(0, -y_size / 2, height=screw_height, 0, screw_r=mount_screw_d/2);
screw_hole(0, y_size / 2, height=screw_height, 0, screw_r=mount_screw_d/2);

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
            mount_bar_x_size + mount_ext / 2,
            screw_r=camera_screw_d / 2,
            wall_r=1.6,
            height=mount_ext);
    }
}
