// =====================================================================
//  Wheel-legged biped  -  parametric OpenSCAD design
//  4x MG996R (2 per leg, both in the body), 2x N20 wheel motors,
//  ESP32-S3 DevKitC-1, MPU6050, MX1508, 2S 18650 pack.  PLA + TPU tyres.
//
//  How to use
//    * Open in OpenSCAD, set `part`, press F6, export STL.
//    * Or:  openscad -D 'part="crank"' -o crank.stl wheel_biped.scad
//      part = "assembly" | "chassis" | "deck" | "top" | "pillar" | "crank"
//             | "link_a" | "link_b" | "spacer" | "wheel_hub" | "tire" | "visor"
//    * In "assembly" you can change `wheel_depth` to see the robot crouch/stand.
//
//  Axes: X = along the wheel axle (left/right), Y = forward, Z = up.
//  Origin = midway between the two servo shafts of a leg (body centre).
//
//  LEG = five-bar linkage (side view, front is to the right):
//
//      rear servo o----o front servo        crank  L1 (on the servo horn)
//                /      \
//         ER   o        o  EF               link A  L2 (EF -> P)
//               \      /                    link B  LB (ER -> P -> W), carries the motor
//                 \  /
//                  P   (pin)
//                  |
//                  W   (wheel axle / N20 motor)
// =====================================================================

part = "assembly";
wheel_depth = 110;         // assembly preview only: wheel axle this far below the servo shafts (70..145)

/* ---------------- fit tolerances ---------------- */
tol          = 0.2;
pin_hole     = 3.3;        // M3 pin joints (bolt + nyloc nut). 3.2 = tighter, 3.4 = looser
m3_clear     = 3.4;
m3_head_d    = 6.0;
use_inserts  = true;       // M3 heat-set inserts in the chassis posts / pillars. false = 2.8 mm holes
insert_d     = 4.0;
insert_depth = 6.0;
shaft_d      = 3.0;        // N20 shaft
shaft_flat   = 2.5;
shaft_fit    = 0.25;       // extra on the D-hole. 0.10 was too tight on the test print (2026-11-22)

/* ---------------- leg geometry (match sim/leg_kinematics.py!) ---------------- */
A_half       = 13.5;       // half spacing of the two servo shafts (y = +-13.5)
L1           = 55;         // crank
L2           = 75;         // link A, and ER->P on link B
LB           = 100;        // link B, ER -> wheel axle
link_t       = 5;          // thickness of every link plate
link_w       = 10;         // width of links
gap          = 0.6;        // washer gap between stacked links

/* ---------------- MG996R ---------------- */
sv_L         = 40.7;       // body length (along Z here)
sv_W         = 19.7;       // body width  (along Y)
sv_below_tab = 26.5;       // body depth from the underside of the mounting tabs to the bottom
sv_tab_t     = 2.5;
sv_above_tab = 9.5;        // body height from the top of the tabs to the top face
sv_shaft_off = 10.2;       // shaft centre from the near end of the body
sv_tab_len   = 54.5;       // tab tip to tab tip
sv_hole_dz   = 49.5;       // mounting hole spacing along the length
sv_hole_dy   = 10.0;       // mounting hole spacing across
sv_screw     = 2.5;        // pilot hole for the screws that come with the servo
horn_d       = 21.0;       // round disc horn that comes with generic MG996R kits (~20.5-21 mm)
horn_t       = 2.6;        // disc thickness (recess depth in the crank)
horn_hole_r  = 7.5;        // (unused now: the crank has radial slots that fit any hole circle)
horn_screw_d = 2.0;        // pilot for small screws through the horn into the crank
horn_stack   = 5.0;        // servo top face -> top of the horn

/* ---------------- N20 motor (ADJUSTABLE: measure yours) ---------------- */
// Jonathan's motors (datasheet drawing, 2026-11-22): 12 x 10 mm, gearbox 9 mm, can 15 mm, D-shaft 10 mm.
// The pocket goes right through link B, so the motor can sit at any depth and an
// off-centre output shaft still clears.
n20_w        = 12.0;       // across the round sides
n20_h        = 10.0;       // across the flats
motor_tol    = 0.2;        // extra room in the pocket; the pinch bolt squeezes it tight
grip_len     = 16;         // how much of the motor the block holds (gearbox + front of can)

/* ---------------- wheel (same as the balancing bot) ---------------- */
tire_od      = 67.0;
tire_t       = 3.5;
tire_stretch = 0.8;
wheel_w      = 10.0;
hub_len      = 8.5;
lip_w        = 1.2;
lip_h        = 1.5;
spokes       = 6;

/* ---------------- body ---------------- */
wall         = 4;          // side plates that hold the servos
floor_t      = 3;
box_y        = 64;         // chassis depth (front-back)
box_top      = 18;         // chassis rim height above the shafts
deck_w       = 104;  deck_d = 84;  deck_t = 3;
top_w        = 104;  top_d  = 84;  top_t  = 4;
pillar_d     = 9;
pillar_h     = 40;
post_xy      = [24, 27];   // chassis corner posts (deck bolts)
pil_xy       = [44, 34];   // deck -> top pillars
esp_len      = 63.0;  esp_w = 25.4;  esp_post_h = 14;
batt_l       = 78;  batt_w = 41;     // 2x18650 holder footprint (measure yours)

$fn = 48;

// ---------------- derived ----------------
in_x      = sv_below_tab + 1;            // inner face of side plates (x)
out_x     = in_x + wall;                 // outer face (servo tabs rest here)
sv_top_x  = out_x + sv_tab_t + sv_above_tab;
crank_x0  = sv_top_x + horn_stack - horn_t;   // crank inner face (horn sits in the recess)
planeA_x0 = crank_x0 + horn_t + link_t + gap;
planeB_x0 = planeA_x0 + link_t + gap;
wheel_x0  = planeB_x0 + link_t + 1.0;
box_bot   = -(sv_L - sv_shaft_off) - (sv_tab_len - sv_L)/2 - floor_t - 1;
sv_cz     = sv_shaft_off - sv_L/2;       // servo body centre z (shaft at z = 0)
deck_z    = box_top;
top_z     = deck_z + deck_t + pillar_h;
total_w   = 2*(wheel_x0 + wheel_w);

echo(str("chassis outer x = +-", out_x, "  crank plane x0 = ", crank_x0, "  overall width = ", total_w, " mm"));
echo(str("chassis bottom z = ", box_bot, "  top plate z = ", top_z));

// ---------------------------------------------------------------------
module rounded_box(size, r=3) {
    hull() for (sx=[-1,1], sy=[-1,1])
        translate([sx*(size[0]/2-r), sy*(size[1]/2-r), 0]) cylinder(r=r, h=size[2]);
}
// ---------------- styling helpers ----------------
module oct2d(w, d, c) {    // rectangle with 45-degree cut corners
    polygon([[-w/2+c,-d/2],[w/2-c,-d/2],[w/2,-d/2+c],[w/2,d/2-c],[w/2-c,d/2],[-w/2+c,d/2],[-w/2,d/2-c],[-w/2,-d/2+c]]);
}
module bevel_plate(w, d, h, c, b=1.2) {   // faceted plate with a bevelled top edge
    hull() {
        linear_extrude(h - b) oct2d(w, d, c);
        linear_extrude(h) oct2d(w - 2*b, d - 2*b, c);
    }
}
module bone2d(len, we, wm) {   // link outline: wide at the pins, slim in the middle
    hull() { circle(d=we); translate([0, len/2]) circle(d=wm); }
    hull() { translate([0, len/2]) circle(d=wm); translate([0, len]) circle(d=we); }
}
module groove(y0, y1, depth=0.8, w=2.4, top=link_t) {   // decorative channel on the top face
    hull() for (y=[y0, y1]) translate([0, y, top - depth]) cylinder(d=w, h=depth + 1, $fn=24);
}
module mirror4(p) { for (sx=[-1,1], sy=[-1,1]) translate([sx*p[0], sy*p[1], 0]) children(); }
module ziptie_pair(spacing, len=3.4, w=1.8, h=20) {
    for (s=[-1,1]) translate([s*spacing/2, 0, -1]) cube([w, len, h], center=true);
}
module screw_hole_into_part(depth=insert_depth) {
    if (use_inserts) {
        translate([0,0,-depth]) cylinder(d=insert_d, h=depth+0.01);
        translate([0,0,-16]) cylinder(d=m3_clear-0.2, h=16);
    } else translate([0,0,-16]) cylinder(d=2.8, h=16.01);
}

// ---------------- five-bar kinematics (same as sim/leg_kinematics.py) ----------------
function c2(c0, r0, c1, r1, s) =
    let(d = norm(c1-c0), a = (r0*r0 - r1*r1 + d*d)/(2*d), h = sqrt(max(r0*r0 - a*a, 0)),
        m = c0 + a*(c1-c0)/d, p = [-(c1-c0)[1], (c1-c0)[0]]/d) m + s*h*p;
function pickmin0(p, q) = p[0] < q[0] ? p : q;
function pickmax0(p, q) = p[0] > q[0] ? p : q;
SF = [ A_half, 0];  SR = [-A_half, 0];
function leg_ik(W) =
    let(ER = pickmin0(c2(SR, L1, W, LB, 1), c2(SR, L1, W, LB, -1)),
        P  = ER + (W - ER)*(L2/LB),
        EF = pickmax0(c2(SF, L1, P, L2, 1), c2(SF, L1, P, L2, -1)))
    [EF, ER, P];
function ang(from, to) = atan2(to[0]-from[0], -(to[1]-from[1]));   // from straight down, + = forward

// =====================================================================
//  CHASSIS: the servo box. Servos slide in from the outside, tabs on the side plates.
// =====================================================================
servo_clear = 0.5;   // per side. 0.2 was too tight on the first chassis print (2026-11-22)
cable_notch = [7, 7];  // width x depth of the cable notch at both ends of each opening
module servo_cut() {
    // in the side plate's local frame: plate in YZ, servo centre at (y, z)
    cube([wall*3, sv_W + 2*servo_clear, sv_L + 2*servo_clear], center=true);
    // notches at both short ends so the cable and its grommet slide through
    for (s=[-1,1]) translate([0, 0, s*(sv_L/2 + servo_clear + cable_notch[1]/2 - 0.5)])
        cube([wall*3, cable_notch[0], cable_notch[1] + 1], center=true);
}
module chassis_envelope() {   // faceted outer shape: chamfered corners and bottom edge
    hull() {
        translate([0,0,box_bot]) linear_extrude(0.01) oct2d(2*out_x - 6, box_y - 6, 3);
        translate([0,0,box_bot + 3]) linear_extrude(box_top - box_bot - 3) oct2d(2*out_x, box_y, 6);
    }
}
module chassis() {
    difference() {
        intersection() { chassis_envelope(); union() {
            // floor
            translate([0,0,box_bot]) rounded_box([2*out_x, box_y, floor_t], 3);
            // side plates
            for (s=[-1,1]) translate([s*(in_x + wall/2), 0, (box_bot + box_top)/2])
                cube([wall, box_y, box_top - box_bot], center=true);
            // front / back walls (low, leave room for wires)
            for (s=[-1,1]) translate([0, s*(box_y/2 - 1.5), (box_bot + box_top)/2])
                cube([2*in_x, 3, box_top - box_bot], center=true);
            // corner posts for the deck bolts
            mirror4(post_xy) translate([0,0,box_bot]) cylinder(d=9, h=box_top - box_bot);
            // bosses for the servo screws (inside, behind the tabs)
            for (s=[-1,1], sy=[-1,1], hz=[-1,1], hy=[-1,1])
                translate([s*(in_x - 3), sy*A_half + hy*sv_hole_dy/2, sv_cz + hz*sv_hole_dz/2])
                    rotate([0,90,0]) cylinder(d=6.5, h=6, center=true);
        } }
        // hex vents in the front and back walls
        for (s=[-1,1], r=[0:3], c=[-2:2]) if (!(r%2==1 && c==2))
            translate([c*7 + (r%2)*3.5, s*(box_y/2 - 1.5), -11 + r*6.1])
                rotate([90,0,0]) rotate([0,0,30]) cylinder(d=6.6, h=6, center=true, $fn=6);
        // servo openings
        for (s=[-1,1], sy=[-1,1]) translate([s*(in_x + wall/2), sy*A_half, sv_cz]) servo_cut();
        // servo screw pilots
        for (s=[-1,1], sy=[-1,1], hz=[-1,1], hy=[-1,1])
            translate([s*(in_x), sy*A_half + hy*sv_hole_dy/2, sv_cz + hz*sv_hole_dz/2])
                rotate([0,90,0]) cylinder(d=sv_screw, h=24, center=true);
        // deck bolts into the posts
        mirror4(post_xy) translate([0,0,box_top]) screw_hole_into_part();
        // wire exits: big slots in front and back walls
        for (s=[-1,1]) translate([0, s*(box_y/2), box_bot + floor_t + 8]) cube([22, 10, 12], center=true);
        // floor: lightening + wire hole
        translate([0,0,box_bot-1]) rounded_box([20, 30, floor_t+2], 3);
    }
}

// =====================================================================
//  DECK: lid of the chassis. ESP32, MPU6050, MX1508, MP1584, servo UBEC.
// =====================================================================
module esp_post() {
    pw = 8;  pd = esp_w + 4;
    difference() {
        translate([-pw/2, -pd/2, 0]) cube([pw, pd, esp_post_h]);
        translate([-pw, -(esp_w+0.6)/2, esp_post_h-0.8]) cube([2*pw, esp_w+0.6, 2]);
        translate([0, 0, esp_post_h-5]) cube([pw+2, 3.6, 2.2], center=true);
        translate([0, 0, esp_post_h-5]) cube([3.6, pd+2, 2.2], center=true);
    }
}
module deck() {
    difference() {
        union() {
            bevel_plate(deck_w, deck_d, deck_t, 8, 0.8);
            // ESP32 on the back half, pins down
            for (s=[-1,1]) translate([s*(esp_len/2 - 5), -22, deck_t]) esp_post();
            // MPU6050 pad, dead centre (on the pitch axis line)
            translate([0, 6, deck_t]) rounded_box([24, 20, 1.5], 2);
        }
        mirror4(post_xy) {
            translate([0,0,-1]) cylinder(d=m3_clear, h=deck_t+2);
            translate([0,0,deck_t-1.8]) cylinder(d=m3_head_d, h=5);
        }
        mirror4(pil_xy) {
            translate([0,0,-1]) cylinder(d=m3_clear, h=deck_t+2);
            translate([0,0,-1]) cylinder(d=m3_head_d, h=1.8+1);
        }
        // wire pass-throughs to the servos below
        for (sx=[-1,1]) translate([sx*14, 26, -1]) rounded_box([18, 8, deck_t+2], 2);
        translate([0, -2, -1]) rounded_box([14, 8, deck_t+2], 2);
        // MPU: M2 slots (GY-521 holes ~15.2 apart) + zip-tie
        for (sx=[-1,1]) hull() for (dy=[-1,1]) translate([sx*7.6, 12 + dy*1.2, -1]) cylinder(d=2.2, h=10);
        translate([0, 3, deck_t]) rotate([0,0,90]) ziptie_pair(22);
        // MX1508 (front left), UBEC (front right), MP1584 (back, under the ESP32 overhang)
        translate([-36, 22, deck_t]) { ziptie_pair(24); rotate([0,0,90]) ziptie_pair(18); }
        translate([ 36, 22, deck_t]) { ziptie_pair(24); rotate([0,0,90]) ziptie_pair(18); }
        translate([0, -22, deck_t]) rotate([0,0,90]) ziptie_pair(16);
        // side lightening: hex cut-outs
        for (sx=[-1,1], y=[-12, 0, 12]) translate([sx*44, y, -1]) rotate([0,0,30]) cylinder(d=9, h=deck_t+2, $fn=6);
    }
}

// =====================================================================
//  TOP: battery (2x18650) + a 20 mm grid of M3 holes for the arm later
// =====================================================================
module top() {
    difference() {
        bevel_plate(top_w, top_d, top_t, 8, 1.2);
        mirror4(pil_xy) {
            translate([0,0,-1]) cylinder(d=m3_clear, h=top_t+2);
            translate([0,0,top_t-3]) cylinder(d=m3_head_d, h=5);
        }
        // velcro strap slots for the battery holder (2 straps, 20 mm)
        for (sx=[-1,1], sy=[-1,1]) translate([sx*20, sy*(batt_w/2 + 4), -1]) rounded_box([22, 3.2, top_t+2], 1);
        // battery wire hole
        translate([0, 0, -1]) rounded_box([14, 8, top_t+2], 2);
        // arm / accessory mounting grid (front and back strips + sides), M3 at 20 mm pitch
        for (gx=[-40:20:40], gy=[-34, 34]) translate([gx, gy, -1]) cylinder(d=m3_clear, h=top_t+2);
        for (gx=[-46, 46], gy=[-10, 10]) translate([gx, gy, -1]) cylinder(d=m3_clear, h=top_t+2);
    }
}

module pillar(L=pillar_h) {
    difference() {
        linear_extrude(L, twist=60, slices=40) circle(d=pillar_d, $fn=6);   // twisted hex
        translate([0,0,L]) screw_hole_into_part();
        mirror([0,0,1]) screw_hole_into_part();
    }
}

// =====================================================================
//  LEG PARTS  (all drawn flat: length along +Y from the first hole, thickness along +Z)
// =====================================================================
module bar2d(len, w=link_w) { hull() { circle(d=w); translate([0, len]) circle(d=w); } }

module crank() {
    hub = horn_d + 4;
    difference() {
        union() {
            // teardrop: hub flowing into the elbow
            linear_extrude(link_t + horn_t) hull() { circle(d=hub, $fn=96); translate([0, L1]) circle(d=link_w); }
        }
        // lightening window + top channel
        translate([0,0,-1]) linear_extrude(20) hull() { translate([0, 17]) circle(d=6); translate([0, 40]) circle(d=3); }
        translate([0,0,0]) groove(44, L1 - 7, 0.8, 2.4, link_t + horn_t);
        // horn recess on the servo side (z = 0 face)
        translate([0,0,-0.01]) cylinder(d=horn_d + 0.4, h=horn_t + 0.01, $fn=96);
        // screwdriver access for the horn's centre screw
        translate([0,0,-1]) cylinder(d=6.5, h=20);
        // 4 screws through the horn holes into the crank
        // slots instead of holes, so they line up with any disc horn's hole circle (r 5.5 .. 9.5 mm)
        for (a=[0:45:315]) rotate([0,0,a]) hull() for (r=[5.5, 9.5]) translate([r, 0, -1]) cylinder(d=horn_screw_d, h=20);
        // elbow pin
        translate([0, L1, -1]) cylinder(d=pin_hole, h=20);
    }
}

module link_a() {
    difference() {
        linear_extrude(link_t) bone2d(L2, link_w + 1, 8);
        for (y=[0, L2]) translate([0, y, -1]) cylinder(d=pin_hole, h=link_t+2);
        groove(9, L2 - 9);
    }
}

// Link B with the motor block. The motor is pushed in from the block side until its gearbox
// face is flush with the flat side of the link, then the pinch bolt squeezes it.
mb_w = n20_w + motor_tol + 2*3;    // block outer
mb_h = n20_h + motor_tol + 2*3;
module link_b() {
    difference() {
        union() {
            linear_extrude(link_t) bone2d(LB, link_w + 1, 8);
            // motor block around the axle end (chamfered)
            translate([0, LB, 0]) linear_extrude(link_t + grip_len) oct2d(mb_h, mb_w, 2);
            // pinch ears below the axle
            translate([0, LB + mb_w/2 + 3, (link_t + grip_len)/2]) cube([12, 8, link_t + grip_len], center=true);
        }
        for (y=[0, L2]) translate([0, y, -1]) cylinder(d=pin_hole, h=link_t+2);
        groove(9, L2 - 9);
        // motor pocket: n20_h across the link, n20_w along it (flats face the sides)
        translate([0, LB, -1]) rotate([0,0,90]) translate([-(n20_w+motor_tol)/2, -(n20_h+motor_tol)/2, 0])
            cube([n20_w + motor_tol, n20_h + motor_tol, link_t + grip_len + 2]);
        // pinch slot + bolt (M3 x 16 + nut)
        translate([0, LB + mb_w/2 + 2, (link_t + grip_len)/2]) cube([1.6, 14, link_t + grip_len + 2], center=true);
        translate([0, LB + mb_w/2 + 3.5, (link_t + grip_len)/2]) rotate([0,90,0]) cylinder(d=m3_clear, h=20, center=true);
        translate([5, LB + mb_w/2 + 3.5, (link_t + grip_len)/2]) rotate([0,90,0]) cylinder(d=6.4, h=4, $fn=6);
        // wire exit at the back of the block (the motor's tabs stick out here)
    }
}

module spacer() {   // goes on the rear elbow pin, between crank and link B (link A's plane)
    difference() { cylinder(d=8, h=link_t + 2*gap); translate([0,0,-1]) cylinder(d=pin_hole, h=link_t + 3); }
}

// ---------------- wheel hub + TPU tyre (from the balancing bot) ----------------
module d_hole(h) {
    intersection() {
        cylinder(d=shaft_d + shaft_fit, h=h, $fn=40);
        translate([-(shaft_d+1)/2, -(shaft_flat - shaft_d/2) - shaft_fit/2, 0]) cube([shaft_d+1, shaft_d+1, h]);
    }
}
module wheel_hub() {
    rim_d = tire_od - 2*tire_t;  hub_d = 11;  rim_in = rim_d - 9;
    difference() {
        union() {
            difference() { cylinder(d=rim_d, h=wheel_w, $fn=120); translate([0,0,-1]) cylinder(d=rim_in, h=wheel_w+2, $fn=120); }
            for (z=[0, wheel_w - lip_w]) translate([0,0,z]) difference() {
                cylinder(d=rim_d + 2*lip_h, h=lip_w, $fn=120); translate([0,0,-1]) cylinder(d=rim_in, h=lip_w+2, $fn=120); }
            cylinder(d=hub_d, h=wheel_w);
            cylinder(d=hub_d+6, h=3);
            // curved spokes
            for (i=[0:spokes-1]) rotate([0,0,i*360/spokes]) linear_extrude(3)
                for (j=[0:7]) hull() for (k=[j, j+1]) let(f=k/8, r=hub_d/2 + 1 + f*(rim_in/2 - hub_d/2 - 0.5), a=40*f)
                    translate([r*cos(a), r*sin(a)]) circle(d=3.6, $fn=16);
            difference() { cylinder(d=rim_in+1, h=1.6, $fn=120); translate([0,0,-1]) cylinder(d=rim_in-8, h=4, $fn=120); }
        }
        translate([0,0,wheel_w - hub_len]) d_hole(hub_len + 1);
        translate([0,0,wheel_w - hub_len/2]) rotate([90,0,0]) cylinder(d=1.8, h=hub_d);
    }
}
module tire() {
    hub_rim = tire_od - 2*tire_t;  w = wheel_w - 2*lip_w - 0.3;
    difference() {
        cylinder(d=tire_od, h=w, $fn=160);
        translate([0,0,-1]) cylinder(d=hub_rim - tire_stretch, h=w+2, $fn=160);
        // chevron tread: V-shaped grooves pointing forward
        for (i=[0:29]) rotate([0,0,i*12]) for (zz=[[w/2, -1], [w/2, w+1]])
            hull() {
                translate([tire_od/2 - 0.8, -0.6, zz[0]]) cube([2, 1.2, 0.01]);
                rotate([0,0,7]) translate([tire_od/2 - 0.8, -0.6, zz[1]]) cube([2, 1.2, 0.01]);
            }
    }
}

// =====================================================================
//  VISOR: the robot's "face". Bolts to the two front grid holes (x = +-20) of the top plate.
//  The two 16 mm holes take an HC-SR04 if you ever want distance sensing (optional).
// =====================================================================
module visor() {
    fw = 62; fh = 28; ft = 2.5;
    difference() {
        union() {
            // foot
            translate([0, -5, 0]) linear_extrude(3) oct2d(52, 10, 3);
            // face
            rotate([90,0,0]) translate([0, 0, -ft]) linear_extrude(ft)
                polygon([[-fw/2+6, 0], [fw/2-6, 0], [fw/2, 8], [fw/2-4, fh], [-fw/2+4, fh], [-fw/2, 8]]);
            // brow ridge
            translate([0, ft, fh - 3]) rotate([90,0,0]) linear_extrude(1.5) polygon([[-fw/2+4, 0], [fw/2-4, 0], [fw/2-6, 3], [-fw/2+6, 3]]);
            // gussets
            for (sx=[-1,1]) translate([sx*24 - 1, 0, 0]) rotate([0,-90,0]) mirror([0,0,1])
                linear_extrude(2) polygon([[0, 0], [12, 0], [0, -8]]);
        }
        for (sx=[-1,1]) translate([sx*13, ft + 1, 15]) rotate([90,0,0]) cylinder(d=16.6, h=ft + 4, $fn=64);
        for (sx=[-1,1]) translate([sx*20, -5, -1]) cylinder(d=m3_clear, h=6);
    }
}

// =====================================================================
//  Assembly preview. Colours = Jonathan's filaments: white body, blue accents, grey links.
// =====================================================================
// place a flat part (drawn along +Y, thickness +Z) into the leg plane:
// part's +Z -> +X (outward), its first hole at p (y,z), pointing toward q
module place_link(x0, p, q, flip=false) {
    u = (q - p)/norm(q - p);                    // direction in the (Y, Z) plane
    multmatrix([[0, 0, 1, x0], [u[1], u[0], 0, p[0]], [-u[0], u[1], 0, p[1]], [0, 0, 0, 1]])
        mirror([0,0,flip?1:0]) children();
}
module servo_dummy() {   // x = 0 at the outer face of the side plate
    color("#1c1c1c") translate([-sv_below_tab, -sv_W/2, sv_cz - sv_L/2]) cube([sv_below_tab + sv_tab_t + sv_above_tab, sv_W, sv_L]);
    color("#1c1c1c") translate([0, -sv_W/2, sv_cz - sv_tab_len/2]) cube([sv_tab_t, sv_W, sv_tab_len]);
    color("white") translate([sv_top_x - out_x, 0, 0]) rotate([0,90,0]) cylinder(d=horn_d, h=horn_stack);
}
module leg(depth) {
    W = [0, -depth];
    k = leg_ik(W);  EF = k[0];  ER = k[1];  P = k[2];
    for (sy=[-1,1]) translate([out_x, sy*A_half, 0]) servo_dummy();
    // cranks: the horn recess faces the servo (part z=0 -> x = crank_x0)
    color("#2f6fd0") place_link(crank_x0, SF, EF) crank();
    color("#2f6fd0") place_link(crank_x0, SR, ER) crank();
    color("#8d949c") place_link(planeA_x0, EF, P) link_a();
    color("#8d949c") place_link(planeB_x0 + link_t, ER, W, flip=true) link_b();
    color("#8d949c") translate([planeA_x0 - gap/2, ER[0], ER[1]]) rotate([0,90,0]) spacer();
    // motor + wheel
    color("silver") translate([planeB_x0 + link_t - 13, W[0], W[1]]) cube([26, n20_h, n20_w], center=true);
    color("#2f6fd0") translate([wheel_x0 + wheel_w, W[0], W[1]]) rotate([0,-90,0]) wheel_hub();
    color("#212529") translate([wheel_x0 + wheel_w - lip_w - 0.15, W[0], W[1]]) rotate([0,-90,0]) tire();
}
module assembly() {
    color("#f1f3f5") chassis();
    leg(wheel_depth);
    mirror([1,0,0]) leg(wheel_depth);
    translate([0,0,deck_z]) color("#f1f3f5") deck();
    mirror4(pil_xy) translate([0,0,deck_z + deck_t]) color("#2f6fd0") pillar();
    translate([0,0,top_z]) color("#f1f3f5") top();
    translate([0, 39, top_z + top_t]) color("#2f6fd0") visor();
    color("Black") translate([-esp_len/2, -22 - esp_w/2, deck_z + deck_t + esp_post_h]) cube([esp_len, esp_w, 1.6]);
    color("Navy") translate([-10.5, -2, deck_z + deck_t + 1.5]) cube([21, 16, 1.6]);
    // two 4xAA holders stacked on the top plate
    for (k=[0,1]) color(k ? "#3a3f45" : "#4a5058") translate([-31, -29 - 4, top_z + top_t + k*15.5]) cube([62, 58, 15]);
    // floor marker
    %translate([0,0,-wheel_depth - tire_od/2 - 0.5]) cube([220, 220, 1], center=true);
}

// ---------------------------------------------------------------------
if (part == "assembly")  assembly();
if (part == "chassis")   translate([0,0,-box_bot]) chassis();
if (part == "deck")      deck();
if (part == "top")       top();
if (part == "pillar")    pillar();
if (part == "crank")     translate([0,0,link_t + horn_t]) rotate([180,0,0]) crank();   // horn recess up: no bridging
if (part == "link_a")    link_a();
if (part == "link_b")    link_b();
if (part == "spacer")    spacer();
if (part == "wheel_hub") wheel_hub();
if (part == "tire")      tire();
if (part == "visor")     translate([0,0,2.5]) rotate([-90,0,0]) visor();   // face down
