// ============================================================
//  ROVER CONTROL  —  PID-Driven Autonomous Rover Game
//  Author  : [Your Name]
//  Degree  : BSc Automatic Control & Robotics
//
//  Compatible : C++11, OnlineGDB (Linux, no raw-mode needed)
//  Input mode : type a command + ENTER each turn (turn-based)
//  Concepts   : PID Controller, State Machine, Sensor Sim,
//               ASCII Renderer, Fixed Timestep Simulation
// ============================================================

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>

// ─────────────────────────────────────────────
//  Portable clamp
// ─────────────────────────────────────────────
template<typename T>
static T myclamp(T v, T lo, T hi){
    return v<lo?lo:(v>hi?hi:v);
}

// ─────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────
static const int   MAP_W  = 58;
static const int   MAP_H  = 20;
static const float MY_PI  = 3.14159265f;
// Simulation sub-steps run per player command
// so physics feels smooth even in turn-based mode
static const int   STEPS_PER_CMD = 12;
static const float DT            = 0.06f;  // seconds per sub-step

// ─────────────────────────────────────────────
//  Vec2
// ─────────────────────────────────────────────
struct Vec2 {
    float x, y;
    Vec2() : x(0.f), y(0.f) {}
    Vec2(float ax, float ay) : x(ax), y(ay) {}
    Vec2 operator+(const Vec2& o) const { return Vec2(x+o.x, y+o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x-o.x, y-o.y); }
    float len() const { return std::sqrt(x*x + y*y); }
};

// ─────────────────────────────────────────────
//  PID Controller
// ─────────────────────────────────────────────
class PID {
public:
    float kp, ki, kd, out_min, out_max;
    PID(float p,float i,float d,float lo,float hi)
        : kp(p),ki(i),kd(d),out_min(lo),out_max(hi),
          intg(0.f),prev(0.f){}
    float update(float sp, float meas, float dt){
        float e=sp-meas;
        intg+=e*dt;
        if(ki!=0.f) intg=myclamp(intg,out_min/ki,out_max/ki);
        float d2=(dt>0.f)?(e-prev)/dt:0.f;
        prev=e;
        return myclamp(kp*e+ki*intg+kd*d2,out_min,out_max);
    }
    void reset(){ intg=0.f; prev=0.f; }
    float intg, prev;
};

// ─────────────────────────────────────────────
//  Tile types
// ─────────────────────────────────────────────
enum Tile { EMPTY='.', WALL='#', HAZARD='~',
            BOOST='+', WAYPOINT='W', GOAL='G' };

// ─────────────────────────────────────────────
//  State machine
// ─────────────────────────────────────────────
enum RoverState { ST_MANUAL, ST_AUTO, ST_RECHARGE, ST_WIN };

// ─────────────────────────────────────────────
//  Map
// ─────────────────────────────────────────────
struct Map {
    char cells[MAP_H][MAP_W];

    void init(){
        for(int r=0;r<MAP_H;r++)
            for(int c=0;c<MAP_W;c++)
                cells[r][c]=(char)EMPTY;
        // Border
        for(int c=0;c<MAP_W;c++){cells[0][c]=(char)WALL;cells[MAP_H-1][c]=(char)WALL;}
        for(int r=0;r<MAP_H;r++){cells[r][0]=(char)WALL;cells[r][MAP_W-1]=(char)WALL;}
        // Interior walls
        hw(3, 2,22); hw(3,32,48);
        vw(4, 3, 9); vw(32,3, 9);
        hw(9, 4,53);
        vw(9, 9,15); vw(49,9,15);
        hw(15,2,53);
        // Doors
        cells[3][27]=(char)EMPTY;
        cells[9][27]=(char)EMPTY;
        cells[6][9] =(char)EMPTY;
        cells[12][9]=(char)EMPTY;
        cells[12][49]=(char)EMPTY;
        cells[15][26]=(char)EMPTY;
        cells[15][27]=(char)EMPTY;
        cells[15][28]=(char)EMPTY;
        // Swamp
        for(int r=10;r<=14;r++)
            for(int c=11;c<=18;c++)
                if(cells[r][c]==(char)EMPTY) cells[r][c]=(char)HAZARD;
        // Boost pads
        cells[5][38]=(char)BOOST; cells[5][39]=(char)BOOST;
        cells[17][13]=(char)BOOST; cells[17][14]=(char)BOOST;
        // Waypoints (left-to-right so autopilot path is logical)
        cells[1][27]=(char)WAYPOINT;
        cells[7][51]=(char)WAYPOINT;
        cells[17][42]=(char)WAYPOINT;
        // Goal
        cells[17][53]=(char)GOAL;
    }

    bool solid(int r,int c) const {
        if(r<0||r>=MAP_H||c<0||c>=MAP_W) return true;
        return cells[r][c]==(char)WALL;
    }
    char get(int r,int c) const {
        if(r<0||r>=MAP_H||c<0||c>=MAP_W) return (char)WALL;
        return cells[r][c];
    }
private:
    void hw(int r,int c0,int c1){
        for(int c=c0;c<=c1&&c<MAP_W;c++) cells[r][c]=(char)WALL;}
    void vw(int c,int r0,int r1){
        for(int r=r0;r<=r1&&r<MAP_H;r++) cells[r][c]=(char)WALL;}
};

// ─────────────────────────────────────────────
//  Rover
// ─────────────────────────────────────────────
struct Rover {
    Vec2  pos;
    float heading, speed, energy;
    RoverState state;
    PID   steer_pid, throttle_pid;
    std::vector<Vec2> waypoints;
    int   wp_idx, score, collisions;
    float elapsed;
    bool  won;
    std::vector<std::pair<int,int> > trail;

    Rover()
        : pos(2.f,1.f), heading(0.f), speed(0.f), energy(100.f),
          state(ST_MANUAL),
          steer_pid   (2.5f,0.1f,0.8f,-3.f, 3.f),
          throttle_pid(1.2f,0.0f,0.4f,-1.5f,3.f),
          wp_idx(0),score(0),collisions(0),elapsed(0.f),won(false){}

    void init(const Map& map){
        pos=Vec2(2.f,1.f); heading=0.f; speed=0.f; energy=100.f;
        state=ST_MANUAL; wp_idx=0; score=0; collisions=0;
        elapsed=0.f; won=false;
        trail.clear(); steer_pid.reset(); throttle_pid.reset();
        waypoints.clear();
        // Collect waypoints + goal, sort left→right
        for(int r=0;r<MAP_H;r++)
            for(int c=0;c<MAP_W;c++){
                char t=map.cells[r][c];
                if(t==(char)WAYPOINT||t==(char)GOAL)
                    waypoints.push_back(Vec2((float)c+0.5f,(float)r+0.5f));
            }
        std::sort(waypoints.begin(),waypoints.end(),
            [](const Vec2&a,const Vec2&b){return a.x<b.x;});
    }

    float raycast(const Map& map, float angle, float maxd) const {
        float dx=std::cos(angle), dy=std::sin(angle);
        for(float d=0.15f;d<maxd;d+=0.15f)
            if(map.solid((int)(pos.y+dy*d),(int)(pos.x+dx*d))) return d;
        return maxd;
    }

    // One physics sub-step (called STEPS_PER_CMD times per command)
    void step(const Map& map){
        if(state==ST_WIN) return;
        if(state==ST_RECHARGE){
            energy=myclamp(energy+8.f*DT,0.f,100.f);
            speed*=0.7f;
            if(energy>30.f) state=ST_MANUAL;
            return;
        }
        // Autopilot PID
        if(state==ST_AUTO && wp_idx<(int)waypoints.size()){
            Vec2  tgt=waypoints[wp_idx];
            Vec2  delta=tgt-pos;
            float dist=delta.len();
            float wh=std::atan2(delta.y,delta.x);
            float herr=wh-heading;
            while(herr> MY_PI) herr-=2.f*MY_PI;
            while(herr<-MY_PI) herr+=2.f*MY_PI;
            float av=steer_pid.update(0.f,-herr,DT);
            float wv=(dist>0.5f)?2.6f:1.f;
            float fwd=raycast(map,heading,10.f);
            if(fwd<2.5f) wv*=fwd/2.5f;
            float la=throttle_pid.update(wv,speed,DT);
            heading+=av*DT;
            speed=myclamp(speed+la*DT,0.f,4.f);
            if(dist<0.8f){
                score+=50; wp_idx++;
                steer_pid.reset(); throttle_pid.reset();
            }
            if(wp_idx>=(int)waypoints.size()){ state=ST_WIN; won=true; }
        }

        char tile=map.get((int)pos.y,(int)pos.x);
        bool on_hz=(tile==(char)HAZARD);
        bool on_bt=(tile==(char)BOOST);
        float drain=std::fabs(speed)*1.5f*DT;
        if(on_hz) drain*=2.5f;
        if(on_bt){ speed=myclamp(speed+1.8f*DT,-2.f,5.5f); drain*=0.3f; }
        energy=myclamp(energy-drain,0.f,100.f);
        if(energy<=0.f){ state=ST_RECHARGE; return; }

        float nx=pos.x+std::cos(heading)*speed*DT;
        float ny=pos.y+std::sin(heading)*speed*DT;
        int nr=(int)ny, nc=(int)nx;
        if(map.solid(nr,nc)){
            speed*=-0.3f; collisions++;
        } else {
            if(map.get(nr,nc)==(char)HAZARD) speed*=0.93f;
            pos=Vec2(nx,ny);
        }
        while(heading> MY_PI) heading-=2.f*MY_PI;
        while(heading<-MY_PI) heading+=2.f*MY_PI;
        if(state==ST_MANUAL) speed*=0.95f;

        int row=(int)pos.y, col=(int)pos.x;
        if(trail.empty()||trail.back()!=std::make_pair(row,col))
            trail.push_back(std::make_pair(row,col));
        if((int)trail.size()>20) trail.erase(trail.begin());

        elapsed+=DT;
        score+=(int)(std::fabs(speed)*0.3f);
        if(map.get((int)ny,(int)nx)==(char)GOAL){ state=ST_WIN; won=true; score+=500; }
    }

    // Apply a player command then run STEPS_PER_CMD physics steps
    // cmd: w=forward  s=reverse  a=left  d=right  q=brake  p=autopilot  .=wait
    std::string apply_cmd(char cmd, const Map& map){
        std::string feedback;
        if(cmd=='p'||cmd=='P'){
            if(state==ST_MANUAL){
                state=ST_AUTO; feedback="[AUTOPILOT ON]";
                steer_pid.reset(); throttle_pid.reset();
            } else if(state==ST_AUTO){
                state=ST_MANUAL; speed=myclamp(speed,0.f,3.f);
                feedback="[MANUAL RESTORED]";
            }
        }
        // Apply thrust / steering before steps
        float accel=5.f, turn=2.0f;
        if(state==ST_MANUAL){
            if(cmd=='w'||cmd=='W') speed=myclamp(speed+accel*DT*STEPS_PER_CMD*0.5f,-2.f,5.f);
            if(cmd=='s'||cmd=='S') speed=myclamp(speed-accel*DT*STEPS_PER_CMD*0.5f,-2.f,5.f);
            if(cmd=='a'||cmd=='A') heading-=turn*DT*STEPS_PER_CMD;
            if(cmd=='d'||cmd=='D') heading+=turn*DT*STEPS_PER_CMD;
            if(cmd=='q'||cmd=='Q'){ speed*=0.2f; feedback="[BRAKE]"; }
        }
        for(int i=0;i<STEPS_PER_CMD;i++) step(map);
        return feedback;
    }
};

// ─────────────────────────────────────────────
//  Renderer  — prints map + HUD, no flicker
//  Uses \033[2J\033[H (erase display, home)
//  then a single puts() — fastest clear method
// ─────────────────────────────────────────────
static char hchar(float h){
    while(h<0.f)        h+=2.f*MY_PI;
    while(h>=2.f*MY_PI) h-=2.f*MY_PI;
    int s=(int)((h+MY_PI/4.f)/(MY_PI/2.f))%4;
    const char a[4]={'>','v','<','^'};
    return a[s];
}
static std::string mkbar(float v,float mv,int w,char f){
    int n=myclamp((int)((v/mv)*(float)w),0,w);
    return std::string(n,f)+std::string(w-n,' ');
}
static std::string lpad(const std::string& s,int w){
    if((int)s.size()>=w) return s.substr(0,w);
    return s+std::string(w-(int)s.size(),' ');
}

static void render(const Map& map, const Rover& rov,
                   const std::string& msg, const std::string& feedback){
    // Build entire frame in one string
    std::string frame;
    frame.reserve(3200);

    int rr=(int)rov.pos.y, rc=(int)rov.pos.x;

    for(int r=0;r<MAP_H;r++){
        for(int c=0;c<MAP_W;c++){
            if(r==rr&&c==rc){ frame+=hchar(rov.heading); continue; }
            bool tr=false;
            for(int i=0;i<(int)rov.trail.size();i++)
                if(rov.trail[i].first==r&&rov.trail[i].second==c){tr=true;break;}
            if(tr){frame+='*';continue;}
            char cell=map.get(r,c);
            if     (cell==(char)WALL)     frame+='#';
            else if(cell==(char)HAZARD)   frame+='~';
            else if(cell==(char)BOOST)    frame+='+';
            else if(cell==(char)WAYPOINT) frame+='W';
            else if(cell==(char)GOAL)     frame+='G';
            else                          frame+=' ';
        }
        frame+='\n';
    }

    frame+=std::string(MAP_W,'-'); frame+='\n';

    std::ostringstream ss;
    const char* stlbl[]={"MANUAL","AUTOPILOT","RECHARGING!","GOAL REACHED"};
    ss<<"STATE:"<<std::setw(12)<<stlbl[rov.state]
      <<"  SCORE:"<<std::setw(6)<<rov.score
      <<"  TIME:"<<std::fixed<<std::setprecision(1)<<rov.elapsed<<"s";
    frame+=lpad(ss.str(),MAP_W); frame+='\n';

    ss.str("");ss.clear();
    ss<<"ENERGY["<<mkbar(rov.energy,100.f,20,'|')<<"]"
      <<std::setw(4)<<(int)rov.energy<<"% "
      <<"SPEED["<<mkbar(std::fabs(rov.speed),5.f,12,'=')<<"]"
      <<std::fixed<<std::setprecision(1)<<rov.speed;
    frame+=lpad(ss.str(),MAP_W); frame+='\n';

    ss.str("");ss.clear();
    ss<<"FWD:"<<std::setw(4)<<std::fixed<<std::setprecision(1)
      <<rov.raycast(map,rov.heading,10.f)
      <<"  WP:"<<rov.wp_idx<<"/"<<(int)rov.waypoints.size()
      <<"  COLLISIONS:"<<rov.collisions
      <<"  HDG:"<<std::setprecision(2)<<rov.heading<<"rad";
    frame+=lpad(ss.str(),MAP_W); frame+='\n';

    ss.str("");ss.clear();
    ss<<"PID kp="<<rov.steer_pid.kp<<" ki="<<rov.steer_pid.ki
      <<" kd="<<rov.steer_pid.kd
      <<"  integral="<<std::setprecision(3)<<rov.steer_pid.intg;
    frame+=lpad(ss.str(),MAP_W); frame+='\n';

    char ter=' ';
    char tile=map.get((int)rov.pos.y,(int)rov.pos.x);
    if(tile==(char)HAZARD) frame+=lpad("*** SWAMP — energy draining fast! ***",MAP_W);
    else if(tile==(char)BOOST) frame+=lpad(">>> BOOST PAD — speed surge! <<<",MAP_W);
    else frame+=lpad(" ",MAP_W);
    (void)ter;
    frame+='\n';

    if(!feedback.empty()){ frame+=lpad(feedback,MAP_W); frame+='\n'; }
    frame+=lpad(msg,MAP_W); frame+='\n';
    frame+=lpad("CMD> w=fwd s=rev a=left d=right q=brake p=autopilot .,ENTER=wait x=quit",MAP_W);
    frame+='\n';

    // Erase screen + home cursor, then single write
    std::printf("\033[2J\033[H");
    std::fwrite(frame.c_str(), 1, frame.size(), stdout);
    std::fflush(stdout);
}

// ─────────────────────────────────────────────
//  Title screen
// ─────────────────────────────────────────────
static void title(){
    std::printf("\033[2J\033[H");
    std::cout
        <<"  ################################################\n"
        <<"  #        R O V E R   C O N T R O L            #\n"
        <<"  #     PID-Driven Autonomous Navigation         #\n"
        <<"  #  BSc Automatic Control & Robotics  Project   #\n"
        <<"  ################################################\n\n"
        <<"  GOAL  : reach all [W] waypoints then the [G] goal.\n\n"
        <<"  HOW TO PLAY (type one letter + ENTER each turn)\n"
        <<"    w  — accelerate forward\n"
        <<"    s  — reverse / brake\n"
        <<"    a  — turn left\n"
        <<"    d  — turn right\n"
        <<"    q  — emergency brake\n"
        <<"    p  — toggle PID autopilot ON/OFF\n"
        <<"    .  — wait (coast one tick)\n"
        <<"    r  — restart\n"
        <<"    x  — quit\n\n"
        <<"  LEGEND\n"
        <<"    #=wall  ~=swamp(drain) +=boost  W=waypoint G=goal\n"
        <<"    * =trail    > v < ^ = rover (heading arrow)\n\n"
        <<"  TIP: press p then just hit ENTER repeatedly to watch\n"
        <<"       the PID autopilot drive the rover for you!\n\n"
        <<"  Press ENTER to start...\n";
    std::string d; std::getline(std::cin,d);
}

// ─────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────
int main(){
    std::srand((unsigned)std::time(0));
    title();

    Map   map;
    Rover rover;
    bool  running=true;

    while(running){
        map.init();
        rover.init(map);

        std::string msg     ="Type a command + ENTER. Try 'p' for autopilot!";
        std::string feedback="";

        while(running && !rover.won){
            render(map, rover, msg, feedback);
            feedback="";

            std::string line;
            if(!std::getline(std::cin, line)){
                running=false; break;
            }

            if(line.empty()){
                // Just ENTER — coast
                for(int i=0;i<STEPS_PER_CMD;i++) rover.step(map);
                msg="[coasting]";
                continue;
            }

            char cmd=line[0];
            if(cmd=='x'||cmd=='X'){ running=false; break; }
            if(cmd=='r'||cmd=='R') break;  // restart

            // Update status message based on surroundings
            char tile=map.get((int)rover.pos.y,(int)rover.pos.x);
            if(rover.state==ST_RECHARGE)
                msg="Battery flat — recharging. Keep pressing ENTER.";
            else if(tile==(char)HAZARD)
                msg="SWAMP! Energy draining 2.5x. Escape quickly.";
            else if(tile==(char)BOOST)
                msg="BOOST PAD! Speed surge, low drain.";
            else if(rover.raycast(map,rover.heading,10.f)<1.5f)
                msg="Wall close ahead — steer or reverse!";
            else if(rover.state==ST_AUTO)
                msg="Autopilot active. ENTER=advance  p=take manual control.";
            else
                msg="Manual. w/s/a/d to drive, p=autopilot.";

            feedback=rover.apply_cmd(cmd, map);
        }

        if(!running) break;

        if(rover.won){
            std::printf("\033[2J\033[H");
            std::cout
                <<"\n  ======  MISSION COMPLETE!  ======\n\n"
                <<"  Score      : "<<rover.score<<"\n"
                <<"  Time       : "<<std::fixed<<std::setprecision(1)<<rover.elapsed<<"s\n"
                <<"  Collisions : "<<rover.collisions<<"\n"
                <<"  Waypoints  : "<<rover.wp_idx<<"/"<<(int)rover.waypoints.size()<<"\n\n"
                <<"  ENTER = play again     x+ENTER = quit\n";
            std::string ch; std::getline(std::cin,ch);
            if(!ch.empty()&&(ch[0]=='x'||ch[0]=='X')) running=false;
        }
    }

    std::printf("\033[2J\033[H");
    std::cout
        <<"  Thanks for playing ROVER CONTROL!\n"
        <<"  Final score: "<<rover.score<<"\n\n"
        <<"  -- BSc Automatic Control & Robotics --\n\n";
    return 0;
}