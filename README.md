 UAV-X: Resilient BVLOS Swarm — Stage 1 Proof-of-Concept                    
                                                                              
  Stage 1 submission for the PUSHPAK Grand Challenge 2026,                    
  UAV-X: Resilient BVLOS Swarm Challenge.                                     
                                                                              
  Hosted by IIT Bombay Techfest 2026-27, in collaboration with IISER Bhopal.  
  Funded by the Ministry of Electronics and Information Technology (MeitY),   
  Government of India.                                                        
                                                                              
  --------                                                                    
                                                                              
  ## Overview                                                                 
                                                                              
  This package provides a working proof-of-concept simulation of a            
  resilient BVLOS UAV swarm for disaster response. It demonstrates:           
                                                                              
  1. Multi-drone coordination — Three ArduPilot SITL instances controlled     
  simultaneously from a single Python process via pymavlink.                  
  2. Priority-weighted survey — Points of Interest (PoIs) ranked              
  P1 (life-critical), P2 (infrastructure), P3 (situational), and              
  surveyed in priority order.                                                 
  3. Dedicated relay UAV — A relay drone maintains the link between           
  survey drones and the Ground Control Station (GCS).                         
  4. Quality-driven relay repositioning — The relay autonomously moves        
  when the simulated Packet Delivery Ratio (PDR) drops below threshold.       
  5. Battery-aware return-to-home — Drones autonomously RTB when              
  battery falls below 25 percent.                                             
  6. Structured decision logging — All autonomy decisions are timestamped     
  and printed for reproducibility.                                            
                                                                              
  The full three-tier architecture (GCS / Relay / Survey) is documented in    
   docs/architecture.md . Stage 2 will extend this proof-of-concept with      
  FlyNetSim integration, candidate-position search for relay placement, and   
  dynamic role management.                                                    
                                                                              
  --------                                                                    
                                                                              
  ## Requirements                                                             
                                                                              
  • Linux (tested on Arch Linux)                                              
  • Python 3.10 or newer                                                      
  • ArduPilot SITL (installed at  ~/ardupilot )                               
  • QGroundControl (optional, for visualization)                              
                                                                              
  --------                                                                    
                                                                              
  ## Installation                                                             
                                                                              
  Follow these steps exactly. The virtual environment keeps the               
  dependencies isolated from your system Python (Arch Linux blocks            
  system-wide pip installs by design — PEP 668).                              
                                                                              
  ### 1. Clone ArduPilot with submodules                                      
                                                                              
    cd ~                                                                      
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git 
                                                                              
  ### 2. Set up the Python virtual environment                                
                                                                              
    cd uavx-swarm-stage1                                                      
    python3 -m venv .venv                                                     
    source .venv/bin/activate                                                 
    pip install -r requirements.txt                                           
                                                                              
  You will see  (.venv)  appear at the start of your shell prompt. All        
  subsequent Python commands must be run from inside this environment.        
                                                                              
  To activate the venv in a new terminal:                                     
                                                                              
    source ~/uavx-swarm-stage1/.venv/bin/activate                             
                                                                              
  --------                                                                    
                                                                              
  ## How to Launch SITL                                                       
                                                                              
  Open three separate terminals. In each one, run the following               
  commands from  ~/ardupilot/ArduCopter .                                     
                                                                              
  Terminal 1 (Drone 1):                                                       
                                                                              
    cd ~/ardupilot/ArduCopter                                                 
    ../Tools/autotest/sim_vehicle.py -v ArduCopter -I0 --sysid 1 \            
        --console --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551         
                                                                              
  Terminal 2 (Drone 2):                                                       
                                                                              
    cd ~/ardupilot/ArduCopter                                                 
    ../Tools/autotest/sim_vehicle.py -v ArduCopter -I1 --sysid 2 \            
        --console --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14561         
                                                                              
  Terminal 3 (Drone 3):                                                       
                                                                              
    cd ~/ardupilot/ArduCopter                                                 
    ../Tools/autotest/sim_vehicle.py -v ArduCopter -I2 --sysid 3 \            
        --console --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14571         
                                                                              
  Wait for continuous telemetry in each terminal (mode lines, GPS status)     
  before starting any script.                                                 
                                                                              
  Alternative launcher: Run  ./sitl_setup.sh  from this package to            
  launch all three SITL instances in separate gnome-terminal windows.         
                                                                              
  --------                                                                    
                                                                              
  ## How to Run the Scripts                                                   
                                                                              
  Activate the venv first:                                                    
                                                                              
    source ~/uavx-swarm-stage1/.venv/bin/activate                             
                                                                              
  ### Primary script — Disaster Response Demo                                 
                                                                              
    python3 src/disaster_response_demo.py                                     
                                                                              
  Expected output:                                                            
                                                                              
  •  [MAIN]  lines: PoI summary, drone assignments                            
  •  [RELAY]  lines: heartbeat, initial position, PDR to GCS over time        
  •  [SURVEY-N]  lines: PoI arrival, imagery capture, link PDR at each PoI    
  • Automatic return-to-home if battery drops below 25 percent                
                                                                              
  Press  Ctrl+C  when the survey drones complete their PoI lists.             
                                                                              
  ### Supporting scripts                                                      
                                                                              
  Multi-drone waypoint control:                                               
                                                                              
    python3 src/digit_path_demo.py                                            
                                                                              
  Three drones trace digit-shaped ground tracks ("1", "2", "3").              
                                                                              
  Long-range formation:                                                       
                                                                              
    python3 src/triangle_formation.py                                         
                                                                              
  Three drones fly to the vertices of a 3 km equilateral triangle.            
                                                                              
  --------                                                                    
                                                                              
  ## QGroundControl Integration                                               
                                                                              
  Both SITL and the scripts can run alongside QGroundControl.                 
                                                                              
  In QGC, add three UDP comm links:                                           
                                                                              
   Link                                 │ Port                                
  ──────────────────────────────────────┼─────────────────────────────────────
   Drone 1                              │ 14550                               
   Drone 2                              │ 14560                               
   Drone 3                              │ 14570                               
                                                                              
  All three drones appear as distinct icons on the map, each with a           
  unique MAVLink system ID (1, 2, 3).                                         
                                                                              
  --------                                                                    
                                                                              
  ## Software Architecture                                                    
                                                                              
  See  docs/architecture.md  for the full description.                        
                                                                              
  Current implementation status:                                              
                                                                              
   Tier                  │ Status                                             
  ───────────────────────┼────────────────────────────────────────────────────
   Tier 1 — GCS Layer    │ Static role assignment; dynamic management planned 
                         │ for Stage 2                                        
   Tier 2 — Relay Layer  │ Single relay with quality-driven repositioning     
   Tier 3 — Survey Layer │ Priority-weighted PoI traversal                    
                                                                              
  --------                                                                    
                                                                              
  ## File Reference                                                           
                                                                              
   File                            │ Purpose                                  
  ─────────────────────────────────┼──────────────────────────────────────────
    src/disaster_response_demo.py  │ Primary Stage 1 proof-of-concept         
    src/digit_path_demo.py         │ Multi-drone waypoint control             
    src/triangle_formation.py      │ Long-range formation demo                
    docs/architecture.md           │ Full three-tier system design            
    sitl_setup.sh                  │ Convenience launcher for three SITL      
                                   │ instances                                
    requirements.txt               │ Python dependency list                   
                                                                              
  --------                                                                    
                                                                              
  ## Reproducibility Notes                                                    
                                                                              
  • All scripts use local NED coordinates and assume drones start at          
  their home position.                                                        
  • The link-quality model in  disaster_response_demo.py  is a linear         
  distance function with mild noise; Stage 2 will replace this with           
  FlyNetSim (ArduPilot SITL + ns-3) for realistic packet-level FANET          
  modelling.                                                                  
  • The relay repositioning logic is a greedy nudge toward the survey         
  centroid; Stage 2 will implement candidate-position search with             
  PDR-based scoring as described in the technical proposal.                   
                                                                              
  --------                                                                    
                                                                              
  ## Authors                                                                  
                                                                              
  [Your Team Name] — PUSHPAK Grand Challenge 2026, UAV-X Track                
                                                                              
  ## License                                                                  
                                                                              
  Submitted for evaluation under the PUSHPAK Grand Challenge 2026.            
  All code is original work by the team.                                      
