#!/usr/bin/env python3

import streamlit as st
import sys
import os
import json
import time
import threading
import subprocess
from pathlib import Path
from datetime import datetime
import pandas as pd

# HTTP requests for API communication
import requests
import urllib3

# Disable SSL warnings for localhost
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# Mission API Server configuration
MISSION_API_URL = "http://localhost:8000"
API_AVAILABLE = False

def check_api_connection():
    """Check if Mission API Server is running"""
    try:
        response = requests.get(f"{MISSION_API_URL}/api/status", timeout=2)
        return response.status_code == 200
    except:
        return False

def test_airsim_connection():
    """Test AirSim connection via API server"""
    try:
        response = requests.get(f"{MISSION_API_URL}/api/test_airsim", timeout=5)
        if response.status_code == 200:
            result = response.json()
            return result.get('success', False), result
        else:
            return False, f"API request failed: {response.status_code}"
    except requests.exceptions.ConnectionError:
        return False, "Cannot connect to Mission API Server"
    except Exception as e:
        return False, str(e)

# Configure Streamlit page
st.set_page_config(
    page_title=" Drone Mission Control Center",
    page_icon="",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for dark military theme without neon highlights
st.markdown("""
<style>
    /* Global theme - Dark military style */
    .stApp {
        background: linear-gradient(135deg, #0a0f1c 0%, #1a1f2e 50%, #0a0f1c 100%);
        color: #d4d4d4;
    }
    
    /* Hide Streamlit branding */
    #MainMenu, footer, header {visibility: hidden;}
    
    /* Main header styling - military theme */
    .main-header {
        font-size: 2.8rem;
        font-weight: 900;
        color: #ffffff;
        text-align: center;
        padding: 1.5rem 0;
        background: linear-gradient(90deg, #1a1f2e, #2a2f3e, #1a1f2e);
        border: 3px solid #4a5568;
        border-radius: 15px;
        margin-bottom: 2rem;
        text-shadow: 0 0 10px rgba(255, 255, 255, 0.2);
        box-shadow: 
            inset 0 0 30px rgba(74, 85, 104, 0.1),
            0 0 20px rgba(0, 0, 0, 0.5);
        font-family: 'Courier New', monospace;
        letter-spacing: 4px;
    }
    
    /* Section headers styling */
    .section-header {
        font-size: 1.8rem;
        font-weight: bold;
        color: #e2e8f0;
        text-align: center;
        padding: 1rem;
        background: linear-gradient(90deg, #2a2f3e, #3a3f4e, #2a2f3e);
        border: 2px solid #4a5568;
        border-radius: 10px;
        margin-bottom: 1.5rem;
        font-family: 'Courier New', monospace;
        letter-spacing: 2px;
        text-shadow: 0 0 5px rgba(226, 232, 240, 0.3);
    }
    
    /* Mission selection panel */
    .mission-panel {
        background: linear-gradient(135deg, #1a1f2e 0%, #2a2f3e 100%);
        border: 2px solid #4a5568;
        border-radius: 15px;
        padding: 1.5rem;
        margin: 1rem 0;
        box-shadow: 
            inset 0 0 20px rgba(74, 85, 104, 0.05),
            0 0 15px rgba(0, 0, 0, 0.3);
        backdrop-filter: blur(10px);
    }
    
    /* Mission configuration panel */
    .config-panel {
        background: linear-gradient(135deg, #1a1f2e 0%, #2a2f3e 100%);
        border: 2px solid #4a5568;
        border-radius: 15px;
        padding: 1.5rem;
        margin: 1rem 0;
        box-shadow: 
            inset 0 0 20px rgba(74, 85, 104, 0.05),
            0 0 15px rgba(0, 0, 0, 0.3);
        backdrop-filter: blur(10px);
    }
    
    /* Mission description box */
    .mission-description {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%);
        border: 1px solid #4a5568;
        border-radius: 10px;
        padding: 1.2rem;
        margin: 1rem 0;
        color: #cbd5e0;
        font-family: 'Arial', sans-serif;
        line-height: 1.6;
        box-shadow: inset 0 0 15px rgba(74, 85, 104, 0.1);
    }
    
    /* Parameters section */
    .parameters-section {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%);
        border: 1px solid #4a5568;
        border-radius: 10px;
        padding: 1.5rem;
        margin: 1rem 0;
        box-shadow: inset 0 0 15px rgba(74, 85, 104, 0.1);
    }
    
    /* Parameter labels */
    .stSelectbox label, .stSlider label, .stCheckbox label {
        color: #e2e8f0 !important;
        font-weight: bold;
        font-size: 1.1rem;
        text-shadow: 0 0 3px rgba(226, 232, 240, 0.3);
    }
    
    /* Slider styling */
    .stSlider > div > div > div > div {
        background: linear-gradient(90deg, #4a5568, #718096);
        border-radius: 5px;
    }
    
    .stSlider > div > div > div {
        background: #2a2f3e;
        border: 1px solid #4a5568;
        border-radius: 5px;
    }
    
    /* Button styling - military theme */
    .stButton > button {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%);
        color: #e2e8f0 !important;
        border: 2px solid #4a5568;
        border-radius: 8px;
        font-weight: bold;
        font-size: 1rem;
        padding: 0.6rem 1.2rem;
        transition: all 0.3s ease;
        text-shadow: 0 0 3px rgba(226, 232, 240, 0.3);
        box-shadow: 0 0 10px rgba(0, 0, 0, 0.2);
        font-family: 'Courier New', monospace;
    }
    
    .stButton > button:hover {
        background: linear-gradient(135deg, #3a3f4e 0%, #4a4f5e 100%);
        border-color: #718096;
        color: #ffffff !important;
        box-shadow: 0 0 15px rgba(113, 128, 150, 0.2);
        transform: translateY(-1px);
    }
    
    /* Primary button (Launch Mission) */
    .stButton > button[kind="primary"] {
        background: linear-gradient(135deg, #4a5568 0%, #718096 100%) !important;
        color: #ffffff !important;
        border: 2px solid #a0aec0;
        font-weight: 900;
        text-shadow: 0 0 5px rgba(0, 0, 0, 0.5);
    }
    
    .stButton > button[kind="primary"]:hover {
        background: linear-gradient(135deg, #718096 0%, #a0aec0 100%) !important;
        box-shadow: 0 0 20px rgba(113, 128, 150, 0.4);
    }
    
    /* Mission selection buttons */
    .mission-select-btn {
        background: linear-gradient(135deg, #1a1f2e 0%, #2a2f3e 100%) !important;
        border: 1px solid #4a5568 !important;
        border-radius: 8px !important;
        color: #e2e8f0 !important;
        margin: 0.5rem 0 !important;
        padding: 1rem !important;
        width: 100% !important;
        text-align: left !important;
        font-family: 'Arial', sans-serif !important;
    }
    
    .mission-select-btn:hover {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%) !important;
        border-color: #718096 !important;
        box-shadow: 0 0 10px rgba(113, 128, 150, 0.2) !important;
    }
    
    /* Selected mission highlight */
    .mission-selected {
        background: linear-gradient(135deg, #3a3f4e 0%, #4a4f5e 100%) !important;
        border: 2px solid #718096 !important;
        box-shadow: 0 0 15px rgba(113, 128, 150, 0.3) !important;
    }
    
    /* Metrics styling */
    .metric-container {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%);
        border: 1px solid #4a5568;
        border-radius: 8px;
        padding: 1rem;
        text-align: center;
        margin: 0.5rem;
        box-shadow: inset 0 0 10px rgba(74, 85, 104, 0.1);
        color: #e2e8f0;
    }
    
    /* Info boxes */
    .stAlert {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%) !important;
        border: 1px solid #4a5568 !important;
        border-radius: 8px !important;
        color: #cbd5e0 !important;
    }
    
    /* Selectbox styling */
    .stSelectbox > div > div {
        background: #2a2f3e !important;
        border: 1px solid #4a5568 !important;
        border-radius: 5px !important;
        color: #e2e8f0 !important;
    }
    
    /* Checkbox styling */
    .stCheckbox > label > div {
        background: #2a2f3e !important;
        border: 1px solid #4a5568 !important;
        border-radius: 3px !important;
    }
    
    /* Progress bar styling */
    .stProgress > div > div > div > div {
        background: linear-gradient(90deg, #4a5568, #718096) !important;
        border-radius: 5px !important;
    }
    
    /* Sidebar styling */
    .css-1d391kg {
        background: linear-gradient(135deg, #0a0f1c 0%, #1a1f2e 100%) !important;
        border-right: 2px solid #4a5568 !important;
    }
    
    /* Text styling */
    h1, h2, h3, h4, h5, h6 {
        color: #e2e8f0 !important;
        font-family: 'Courier New', monospace !important;
        text-shadow: 0 0 5px rgba(226, 232, 240, 0.2) !important;
    }
    
    /* Status indicators */
    .status-indicator {
        display: inline-block;
        width: 12px;
        height: 12px;
        border-radius: 50%;
        margin-right: 8px;
        box-shadow: 0 0 5px currentColor;
    }
    
    /* Expandable sections */
    .streamlit-expanderHeader {
        background: linear-gradient(135deg, #2a2f3e 0%, #3a3f4e 100%) !important;
        border: 1px solid #4a5568 !important;
        border-radius: 8px !important;
        color: #e2e8f0 !important;
    }
    
    .streamlit-expanderContent {
        background: linear-gradient(135deg, #1a1f2e 0%, #2a2f3e 100%) !important;
        border: 1px solid #4a5568 !important;
        border-top: none !important;
        border-radius: 0 0 8px 8px !important;
    }
    
    /* Success/Error colors - muted military style */
    .stSuccess {
        background: linear-gradient(135deg, #2d3e50 0%, #34495e 100%) !important;
        border: 1px solid #27ae60 !important;
        color: #2ecc71 !important;
    }
    
    .stError {
        background: linear-gradient(135deg, #2d3e50 0%, #34495e 100%) !important;
        border: 1px solid #e74c3c !important;
        color: #ec7063 !important;
    }
    
    .stWarning {
        background: linear-gradient(135deg, #2d3e50 0%, #34495e 100%) !important;
        border: 1px solid #f39c12 !important;
        color: #f7dc6f !important;
    }
</style>
""", unsafe_allow_html=True)

class MissionControlApp:
    def __init__(self):
        self.mission_process = None
        self.mission_active = False
        self.current_mission_type = None
        self.mission_status = {
            'active': False,
            'phase': 'Idle',
            'progress': 0,
            'message': 'Ready to launch mission',
            'type': None
        }
        
        # Initialize session state
        if 'selected_mission' not in st.session_state:
            st.session_state.selected_mission = None
        if 'mission_active' not in st.session_state:
            st.session_state.mission_active = False
        if 'mission_log' not in st.session_state:
            st.session_state.mission_log = []

    def load_mission_types(self):
        """Load available mission types from API server"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/missions", timeout=5)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            st.warning(f"Could not connect to Mission API Server: {str(e)}")
        
        # Fallback mission definitions
        return {
                "box": {
                    "name": "Box Waypoint",
                    "description": "The drone will navigate a four waypoint route creating a box. Corner points will be interpolated with rectangular turns. Final position will return to origin.",
                    "script": "box_mission.py",
                    "parameters": {
                        "box_size": {"type": "float", "default": 400, "min": 100, "max": 1000, "unit": "m"},
                        "altitude": {"type": "float", "default": 20, "min": 5, "max": 100, "unit": "m"},
                        "speed": {"type": "float", "default": 5, "min": 1, "max": 20, "unit": "m/s"},
                        "orbit_radius": {"type": "float", "default": 50, "min": 10, "max": 200, "unit": "m"},
                        "orbit_speed": {"type": "float", "default": 3, "min": 1, "max": 10, "unit": "m/s"},
                        "photos_per_orbit": {"type": "int", "default": 8, "min": 1, "max": 20, "unit": "photos"},
                        "orbit_mode": {"type": "choice", "default": "velocity", "choices": ["waypoint", "velocity"], "unit": ""},
                        "enable_orbits": {"type": "bool", "default": True, "unit": ""}
                    }
                },
                "spiral": {
                    "name": "Spiral Search Mission",
                    "description": "Systematic spiral search pattern for area coverage",
                    "script": "spiral_search_mission.py",
                    "parameters": {
                        "max_radius": {"type": "float", "default": 500, "min": 50, "max": 1000, "unit": "m"},
                        "altitude": {"type": "float", "default": 30, "min": 5, "max": 100, "unit": "m"},
                        "speed": {"type": "float", "default": 8, "min": 1, "max": 20, "unit": "m/s"},
                        "spiral_spacing": {"type": "float", "default": 25, "min": 5, "max": 100, "unit": "m"}
                    }
                },
                "grid": {
                    "name": "Grid Survey Mission", 
                    "description": "Professional survey patterns for mapping and photogrammetry",
                    "script": "grid_mission.py",
                    "parameters": {
                        "width": {"type": "float", "default": 800, "min": 100, "max": 2000, "unit": "m"},
                        "height": {"type": "float", "default": 600, "min": 100, "max": 2000, "unit": "m"},
                        "altitude": {"type": "float", "default": 20, "min": 5, "max": 100, "unit": "m"},
                        "grid_spacing": {"type": "float", "default": 50, "min": 10, "max": 200, "unit": "m"}
                    }
                }
            }

    def render_mission_selection_tab(self):
        """Render the mission selection and configuration tab"""
        st.markdown('<div class="main-header">🚁 MISSION PLANNER</div>', unsafe_allow_html=True)
        
        missions = self.load_mission_types()
        
        col1, col2 = st.columns([1, 2])
        
        # Left Panel: Mission Selection
        with col1:
            st.markdown('<div class="section-header">Mission Selection</div>', unsafe_allow_html=True)
            
            # Mission selection panel container
            # st.markdown('<div class="mission-panel">', unsafe_allow_html=True)
            
            for mission_id, mission_data in missions.items():
                mission_name = mission_data.get('name', mission_id.title())
                mission_desc = mission_data.get('description', 'No description available')
                
                # Create styled mission selection button
                button_class = "mission-selected" if st.session_state.selected_mission == mission_id else "mission-select-btn"
                
                if st.button(f"🎯 {mission_name}", key=f"mission_{mission_id}", use_container_width=True):
                    st.session_state.selected_mission = mission_id
                
                # Show selected indicator
                if st.session_state.selected_mission == mission_id:
                    st.markdown(f'<div style="color: #a0aec0; font-weight: bold; margin: 0.5rem 0;">✅ SELECTED: {mission_name}</div>', unsafe_allow_html=True)
                
                # Mission details in expandable section
                # with st.expander(f"ℹ️ {mission_name} Details", expanded=False):
                #     st.markdown(f'<div class="mission-description">{mission_desc}</div>', unsafe_allow_html=True)
                #     st.markdown(f"**Script:** `{mission_data.get('script', 'N/A')}`")
            
            st.markdown('</div>', unsafe_allow_html=True)
        
        # Right Panel: Mission Configuration
        with col2:
            if st.session_state.selected_mission:
                selected_mission = missions[st.session_state.selected_mission]
                mission_name = selected_mission['name']
                
                # Configuration panel header
                # st.markdown(f'<div class="section-header">{mission_name}</div>', unsafe_allow_html=True)
                
                # Configuration panel container
                # st.markdown('<div class="config-panel">', unsafe_allow_html=True)
                
                # Mission Briefing
                st.markdown("#### 📝 MISSION DESCRIPTION")
                st.markdown(f'<div class="mission-description">{selected_mission["description"]}</div>', unsafe_allow_html=True)
                
                # Add specific guidance for box mission
                if st.session_state.selected_mission == 'box':
                    with st.expander("ℹ️ Orbit Mode Guide", expanded=False):
                        col_mode1, col_mode2 = st.columns(2)
                        with col_mode1:
                            st.markdown("**🎯 Waypoint Mode**")
                            st.write("• Stable, precise movements")
                            st.write("• Stops at each orbital point")
                            st.write("• Better for detailed photography")
                            st.write("• Slightly longer flight time")
                        with col_mode2:
                            st.markdown("**🌊 Velocity Mode**")
                            st.write("• Smooth, continuous motion")
                            st.write("• Fluid circular orbits")
                            st.write("• More cinematic footage")
                            st.write("• Faster mission completion")
                
                # Parameters Configuration
                st.markdown("#### ⚙️ PARAMETERS")
                st.markdown('<div class="parameters-section">', unsafe_allow_html=True)
                
                parameters = {}
                params_config = selected_mission.get('parameters', {})
                
                for param_name, param_config in params_config.items():
                    param_type = param_config.get('type', 'float')
                    default_val = param_config.get('default', 0)
                    min_val = param_config.get('min', 0)
                    max_val = param_config.get('max', 100)
                    unit = param_config.get('unit', '')
                    
                    col_param1, col_param2 = st.columns([3, 1])
                    
                    with col_param1:
                        param_label = f"{param_name.replace('_', ' ').title()}"
                        if param_type == 'float':
                            parameters[param_name] = st.slider(
                                param_label,
                                min_value=float(min_val),
                                max_value=float(max_val),
                                value=float(default_val),
                                step=1.0,
                                key=f"param_{param_name}"
                            )
                        elif param_type == 'int':
                            parameters[param_name] = st.slider(
                                param_label,
                                min_value=int(min_val),
                                max_value=int(max_val),
                                value=int(default_val),
                                step=1,
                                key=f"param_{param_name}"
                            )
                        elif param_type == 'bool':
                            parameters[param_name] = st.checkbox(
                                param_label,
                                value=bool(default_val),
                                key=f"param_{param_name}"
                            )
                        elif param_type == 'choice':
                            choices = param_config.get('choices', [])
                            # Add helpful descriptions for specific parameters
                            if param_name == 'orbit_mode':
                                help_text = "Waypoint: Stable transitions, stops at each point. Velocity: Smooth continuous motion."
                            else:
                                help_text = None
                            
                            parameters[param_name] = st.selectbox(
                                param_label,
                                choices,
                                index=0 if choices else 0,
                                key=f"param_{param_name}",
                                help=help_text
                            )
                    
                    with col_param2:
                        if param_type == 'bool':
                            display_value = "✅ ON" if parameters.get(param_name, default_val) else "❌ OFF"
                            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                            st.markdown(f"**{display_value}**")
                            st.markdown('</div>', unsafe_allow_html=True)
                        elif param_type == 'choice':
                            display_value = str(parameters.get(param_name, default_val)).upper()
                            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                            st.markdown(f"**{display_value}**")
                            st.markdown('</div>', unsafe_allow_html=True)
                        else:
                            value = parameters.get(param_name, default_val)
                            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                            st.markdown(f"**{value} {unit}**")
                            st.markdown('</div>', unsafe_allow_html=True)
                        st.markdown('</div>', unsafe_allow_html=True)
                
                st.markdown('</div>', unsafe_allow_html=True)  # End parameters section
                
                # Estimated Mission Stats
                st.markdown("#### 📊 MISSION ESTIMATES")
                col_stat1, col_stat2, col_stat3, col_stat4 = st.columns(4)
                
                # Calculate estimates based on mission type
                estimated_time, estimated_distance, estimated_photos, estimated_battery = self.calculate_estimates(
                    st.session_state.selected_mission, parameters
                )
                
                with col_stat1:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Flight Time**")
                    st.markdown(f"**{estimated_time:.1f} min**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                with col_stat2:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Distance**")
                    st.markdown(f"**{estimated_distance:.1f} km**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                with col_stat3:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Photos**")
                    st.markdown(f"**{estimated_photos}**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                with col_stat4:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Battery**")
                    st.markdown(f"**{estimated_battery:.0f}%**")
                    st.markdown('</div>', unsafe_allow_html=True)
                
                # Action Buttons
                st.markdown("#### 🎮 MISSION CONTROL")
                col_btn1, col_btn2, col_btn3 = st.columns(3)
                
                with col_btn1:
                    if st.button("👁️ PREVIEW", use_container_width=True):
                        self.preview_mission(st.session_state.selected_mission, parameters)
                
                with col_btn2:
                    if st.button("🚀 LAUNCH MISSION", type="primary", use_container_width=True):
                        if not st.session_state.mission_active:
                            self.launch_mission(st.session_state.selected_mission, parameters)
                        else:
                            st.warning("Mission already active!")
                
                with col_btn3:
                    if st.button("🛑 EMERGENCY STOP", use_container_width=True):
                        if st.session_state.mission_active:
                            self.stop_mission()
                        else:
                            st.info("No active mission to stop")
                
                st.markdown('</div>', unsafe_allow_html=True)  # End config panel
                
                # Store parameters in session state
                st.session_state.mission_parameters = parameters
                
            else:
                # No mission selected state
                st.markdown('<div class="section-header">Select Mission</div>', unsafe_allow_html=True)
                st.markdown('<div class="config-panel">', unsafe_allow_html=True)
                st.markdown('<div class="mission-description">', unsafe_allow_html=True)
                st.markdown("### 🎯 MISSION SELECTION REQUIRED")
                st.markdown("Choose a mission from the **Mission Selection** panel on the left to configure parameters and launch.")
                st.markdown("</div></div>", unsafe_allow_html=True)

    def render_mission_execution_tab(self):
        """Render the mission execution and monitoring tab"""
        st.markdown('<div class="main-header">🚁 MISSION EXECUTION CENTER</div>', unsafe_allow_html=True)
        
        # Add refresh button at the top
        col_title, col_refresh = st.columns([4, 1])
        with col_title:
            st.markdown('<div class="section-header">Monitor Mission Progress</div>', unsafe_allow_html=True)
        with col_refresh:
            if st.button("🔄 REFRESH", use_container_width=True):
                try:
                    st.rerun()  # Streamlit 1.27+
                except AttributeError:
                    st.experimental_rerun()  # Older Streamlit versions
        
        col1, col2 = st.columns([1, 2])
        
        # Left Panel: Mission Status
        with col1:
            st.markdown('<div class="section-header">Mission Status</div>', unsafe_allow_html=True)
            st.markdown('<div class="mission-panel">', unsafe_allow_html=True)
            
            # Get real mission status from API
            try:
                response = requests.get(f"{MISSION_API_URL}/api/status", timeout=2)
                if response.status_code == 200:
                    api_status = response.json()
                    is_active = api_status.get('active', False)
                    
                    # Update session state to match API
                    if st.session_state.mission_active != is_active:
                        st.session_state.mission_active = is_active
                        
                else:
                    api_status = {}
                    is_active = False
            except:
                api_status = {}
                is_active = False
            
            # Status indicators with real data
            status_color = "🟢" if is_active else "🔴"
            status_text = "ACTIVE" if is_active else "INACTIVE"
            st.markdown(f'<div style="font-size: 1.5rem; font-weight: bold; color: #e2e8f0; margin: 1rem 0;"><span class="status-indicator" style="background: {"#27ae60" if is_active else "#e74c3c"};"></span>STATUS: {status_text}</div>', unsafe_allow_html=True)
            
            if is_active:
                # Real mission progress
                progress = self.get_mission_progress()
                st.markdown("**Mission Progress**")
                st.progress(progress / 100)
                st.markdown(f'<div class="metric-container">**{progress:.1f}%** Complete</div>', unsafe_allow_html=True)
                
                # Current phase
                phase = self.get_mission_phase()
                st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                st.markdown("**Current Phase**")
                st.markdown(f"**{phase}**")
                st.markdown('</div>', unsafe_allow_html=True)
                
                # Mission type
                mission_type = api_status.get('type', 'Unknown')
                st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                st.markdown("**Mission Type**")
                st.markdown(f"**{mission_type.upper()}**")
                st.markdown('</div>', unsafe_allow_html=True)
                
                # Mission start time and duration
                start_time = api_status.get('start_time')
                if start_time:
                    duration = time.time() - start_time
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Duration**")
                    st.markdown(f"**{duration/60:.1f} min**")
                    st.markdown('</div>', unsafe_allow_html=True)
                
                # Mission statistics
                st.markdown("#### 📈 REAL-TIME STATS")
                flight_time = self.get_flight_time()
                distance_traveled = self.get_distance_traveled()
                photos_taken = self.get_photos_taken()
                battery_level = self.get_battery_level()
                
                col_stat1, col_stat2 = st.columns(2)
                with col_stat1:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Flight Time**")
                    st.markdown(f"**{flight_time:.1f} min**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Photos**")
                    st.markdown(f"**{photos_taken}**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                with col_stat2:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Distance**")
                    st.markdown(f"**{distance_traveled:.2f} km**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Battery**")
                    st.markdown(f"**{battery_level:.0f}%**")
                    st.markdown('</div>', unsafe_allow_html=True)
                
                # Mission controls
                st.markdown("#### 🎮 MISSION CONTROLS")
                col_ctrl1, col_ctrl2 = st.columns(2)
                
                with col_ctrl1:
                    if st.button("⏸️ PAUSE", use_container_width=True):
                        self.pause_mission()
                
                with col_ctrl2:
                    if st.button("🛑 ABORT", use_container_width=True):
                        self.stop_mission()
                        
                # Real-time message
                message = api_status.get('message', 'No status message')
                if message != 'No status message':
                    st.markdown(f'<div class="mission-description">📡 {message}</div>', unsafe_allow_html=True)
                        
            else:
                st.markdown('<div class="mission-description">', unsafe_allow_html=True)
                st.markdown("### 🏠 STANDBY MODE")
                st.markdown("No active mission. Go to **Mission Planning** tab to start a mission.")
                st.markdown('</div>', unsafe_allow_html=True)
                
                # Show last mission info if available
                if api_status.get('type'):
                    st.markdown("#### 📝 LAST MISSION")
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown(f"**Type:** {api_status.get('type', 'Unknown').upper()}")
                    st.markdown(f"**Status:** {api_status.get('phase', 'Unknown')}")
                    st.markdown('</div>', unsafe_allow_html=True)
            
            st.markdown('</div>', unsafe_allow_html=True)  # End mission panel
        
        # Right Panel: Tactical Display
        with col2:
            st.markdown('<div class="section-header">Tactical Display</div>', unsafe_allow_html=True)
            st.markdown('<div class="config-panel">', unsafe_allow_html=True)
            
            if is_active:
                # Real drone position
                st.markdown("#### 📍 DRONE POSITION")
                
                tactical_data = self.get_tactical_data()
                
                # Position info with real data
                col_pos1, col_pos2, col_pos3 = st.columns(3)
                with col_pos1:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**X Position**")
                    st.markdown(f"**{tactical_data['x']:.1f} m**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                with col_pos2:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Y Position**")
                    st.markdown(f"**{tactical_data['y']:.1f} m**")
                    st.markdown('</div>', unsafe_allow_html=True)
                    
                with col_pos3:
                    st.markdown('<div class="metric-container">', unsafe_allow_html=True)
                    st.markdown("**Altitude**")
                    st.markdown(f"**{tactical_data['z']:.1f} m**")
                    st.markdown('</div>', unsafe_allow_html=True)
                
                # Mission objectives with real data
                st.markdown("#### 🎯 CURRENT OBJECTIVES")
                objectives = self.get_current_objectives()
                objectives_html = '<div class="mission-description"><ul>'
                for objective in objectives:
                    objectives_html += f'<li>{objective}</li>'
                objectives_html += '</ul></div>'
                st.markdown(objectives_html, unsafe_allow_html=True)
                
                # Photo gallery
                st.markdown("#### 📸 RECENT PHOTOS")
                recent_photos = self.get_recent_photos()
                if recent_photos:
                    cols = st.columns(3)
                    for i, photo in enumerate(recent_photos[:3]):
                        with cols[i]:
                            st.image(photo, caption=f"Photo {i+1}", use_column_width=True)
                else:
                    st.markdown('<div class="mission-description">No photos captured yet (coming soon)</div>', unsafe_allow_html=True)
                
                # Mission log with real entries
                st.markdown("#### 📋 MISSION LOG")
                log_container = st.container()
                with log_container:
                    st.markdown('<div class="mission-description" style="max-height: 200px; overflow-y: auto;">', unsafe_allow_html=True)
                    
                    # Show API server logs if available
                    try:
                        response = requests.get(f"{MISSION_API_URL}/api/logs", timeout=2)
                        if response.status_code == 200:
                            log_data = response.json()
                            api_logs = log_data.get('logs', [])
                            for log_entry in api_logs[-5:]:  # Show last 5 API logs
                                st.text(f"API: {log_entry}")
                    except:
                        pass
                    
                    # Show local session logs
                    for log_entry in st.session_state.mission_log[-5:]:  # Show last 5 session logs
                        st.text(f"UI: {log_entry}")
                    
                    if not st.session_state.mission_log:
                        st.text("Mission log will appear here during execution")
                    
                    st.markdown('</div>', unsafe_allow_html=True)
                        
            else:
                st.markdown('<div class="mission-description">', unsafe_allow_html=True)
                st.markdown("### 🖥️ TACTICAL DISPLAY OFFLINE")
                st.markdown("Tactical display will show mission progress when a mission is active.")
                st.markdown('</div>', unsafe_allow_html=True)
                
                # Show connection status when no mission is active
                st.markdown("#### 🔧 SYSTEM STATUS")
                
                # API connection status
                api_connected = check_api_connection()
                api_icon = "🟢" if api_connected else "🔴"
                status_text = "CONNECTED" if api_connected else "OFFLINE"
                st.markdown(f'<div class="metric-container">{api_icon} **Mission API:** {status_text}</div>', unsafe_allow_html=True)
                
                # AirSim connection status
                if api_connected:
                    try:
                        airsim_success, airsim_result = test_airsim_connection()
                        airsim_icon = "🟢" if airsim_success else "🔴"
                        airsim_status = "CONNECTED" if airsim_success else "OFFLINE"
                        st.markdown(f'<div class="metric-container">{airsim_icon} **AirSim:** {airsim_status}</div>', unsafe_allow_html=True)
                        
                        if airsim_success:
                            details = airsim_result.get('details', {})
                            st.markdown(f'<div class="mission-description">**State:** {details.get("landed_state", "Unknown")}</div>', unsafe_allow_html=True)
                    except:
                        st.markdown('<div class="metric-container">🟡 **AirSim:** STATUS UNKNOWN</div>', unsafe_allow_html=True)
                else:
                    st.markdown('<div class="metric-container">🔴 **AirSim:** CANNOT CHECK (API OFFLINE)</div>', unsafe_allow_html=True)
            
            st.markdown('</div>', unsafe_allow_html=True)  # End config panel

    def calculate_estimates(self, mission_type, parameters):
        """Calculate estimated mission statistics"""
        if mission_type == "box":
            box_size = parameters.get('box_size', 400)
            speed = parameters.get('speed', 5)
            orbit_radius = parameters.get('orbit_radius', 50)
            photos_per_orbit = parameters.get('photos_per_orbit', 8)
            
            # Box perimeter + orbital photography
            distance = (4 * box_size + 4 * 2 * 3.14159 * orbit_radius) / 1000  # km
            time_minutes = distance / (speed / 16.67)  # rough estimate
            photos = 4 * photos_per_orbit
            battery = min(90, time_minutes * 5)  # 5% per minute estimate
            
        elif mission_type == "spiral":
            max_radius = parameters.get('max_radius', 500)
            speed = parameters.get('speed', 8)
            spiral_spacing = parameters.get('spiral_spacing', 25)
            
            # Spiral distance approximation
            turns = max_radius / spiral_spacing
            distance = (3.14159 * max_radius * turns) / 1000  # km
            time_minutes = distance / (speed / 16.67)
            photos = int(time_minutes * 0.5)  # 0.5 photos per minute
            battery = min(90, time_minutes * 5)
            
        elif mission_type == "grid":
            width = parameters.get('width', 800)
            height = parameters.get('height', 600)
            speed = parameters.get('speed', 5)
            grid_spacing = parameters.get('grid_spacing', 50)
            
            # Grid pattern distance
            lines = height / grid_spacing
            distance = (lines * width) / 1000  # km
            time_minutes = distance / (speed / 16.67)
            photos = int(time_minutes * 1.0)  # 1 photo per minute
            battery = min(90, time_minutes * 5)
            
        else:
            distance, time_minutes, photos, battery = 1.0, 10.0, 20, 50
            
        return time_minutes, distance, photos, battery

    def preview_mission(self, mission_type, parameters):
        """Preview mission without execution"""
        with st.spinner("Generating mission preview..."):
            time.sleep(2)  # Simulate processing
            
            # Show preview results
            st.success(" Mission preview generated successfully!")
            
            # Add to mission log
            log_entry = f"[{datetime.now().strftime('%H:%M:%S')}] Mission preview: {mission_type}"
            st.session_state.mission_log.append(log_entry)

    def launch_mission(self, mission_type, parameters):
        """Launch the selected mission via API"""
        try:
            # Send launch request to API server
            payload = {
                'mission_type': mission_type,
                'parameters': parameters
            }
            
            response = requests.post(f"{MISSION_API_URL}/api/launch", json=payload, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    st.session_state.mission_active = True
                    st.success(f" {result['message']}")
                else:
                    st.error(f"Mission launch failed: {result.get('message', 'Unknown error')}")
            else:
                st.error(f"API request failed: {response.status_code}")
            
            # Add to mission log
            log_entry = f"[{datetime.now().strftime('%H:%M:%S')}] Mission launch requested: {mission_type}"
            st.session_state.mission_log.append(log_entry)
            
        except requests.exceptions.ConnectionError:
            st.error(" Cannot connect to Mission API Server. Please start the API server first.")
            st.info("Run: python mission_api_server.py")
        except Exception as e:
            st.error(f"Failed to launch mission: {str(e)}")
            st.session_state.mission_active = False

    def stop_mission(self):
        """Stop the current mission via API"""
        try:
            response = requests.post(f"{MISSION_API_URL}/api/stop", timeout=5)
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    st.session_state.mission_active = False
                    st.warning(f" {result['message']}")
                else:
                    st.error(f"Stop failed: {result.get('message', 'Unknown error')}")
            
            # Add to mission log
            log_entry = f"[{datetime.now().strftime('%H:%M:%S')}] Mission stop requested"
            st.session_state.mission_log.append(log_entry)
            
        except requests.exceptions.ConnectionError:
            st.error(" Cannot connect to Mission API Server")
        except Exception as e:
            st.error(f"Failed to stop mission: {str(e)}")

    def pause_mission(self):
        """Pause the current mission"""
        st.info(" Mission pause requested (feature coming soon)")
        
        # Add to mission log
        log_entry = f"[{datetime.now().strftime('%H:%M:%S')}] Mission pause requested"
        st.session_state.mission_log.append(log_entry)

    # Real-time data functions using API server
    def get_mission_progress(self):
        """Get real mission progress from API server"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/status", timeout=2)
            if response.status_code == 200:
                status = response.json()
                return status.get('progress', 0)
        except:
            pass
        return 0
    
    def get_mission_phase(self):
        """Get real mission phase from API server"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/status", timeout=2)
            if response.status_code == 200:
                status = response.json()
                return status.get('phase', 'Idle')
        except:
            pass
        return 'Idle'
    
    def get_flight_time(self):
        """Get real flight time from API server"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/status", timeout=2)
            if response.status_code == 200:
                status = response.json()
                start_time = status.get('start_time')
                if start_time:
                    return (time.time() - start_time) / 60  # Convert to minutes
        except:
            pass
        return 0
    
    def get_distance_traveled(self):
        """Get distance traveled - placeholder for now"""
        return 0  # TODO: Implement when mission provides distance data
    
    def get_photos_taken(self):
        """Get photos taken - placeholder for now"""
        return 0  # TODO: Implement when mission provides photo count
    
    def get_battery_level(self):
        """Get battery level via AirSim through API"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/test_airsim", timeout=2)
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    # For now, return estimated battery based on flight time
                    flight_time = self.get_flight_time()
                    estimated_battery = max(20, 100 - (flight_time * 3))  # 3% per minute estimate
                    return estimated_battery
        except:
            pass
        return 100  # Default to full battery if unknown
    
    def get_tactical_data(self):
        """Get real tactical data from AirSim via API"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/test_airsim", timeout=2)
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    details = result.get('details', {})
                    pos = details.get('position', {})
                    return {
                        'x': pos.get('x', 0),
                        'y': pos.get('y', 0),
                        'z': abs(pos.get('z', 0))  # Convert to positive altitude
                    }
        except:
            pass
        return {'x': 0, 'y': 0, 'z': 0}
    
    def get_current_objectives(self):
        """Get current mission objectives based on real status"""
        try:
            response = requests.get(f"{MISSION_API_URL}/api/status", timeout=2)
            if response.status_code == 200:
                status = response.json()
                phase = status.get('phase', 'Idle')
                mission_type = status.get('type', 'Unknown')
                
                if status.get('active'):
                    objectives = [
                        f"Executing {mission_type} mission",
                        f"Current phase: {phase}",
                        f"Progress: {status.get('progress', 0):.1f}%"
                    ]
                    
                    if mission_type == 'box':
                        objectives.extend([
                            "Navigate box pattern vertices",
                            "Maintain collision detection",
                            "Monitor for emergency conditions"
                        ])
                    elif mission_type == 'spiral':
                        objectives.extend([
                            "Execute spiral search pattern",
                            "Maintain search altitude",
                            "Cover designated area systematically"
                        ])
                    elif mission_type == 'grid':
                        objectives.extend([
                            "Follow grid survey pattern",
                            "Capture mapping photos",
                            "Maintain survey accuracy"
                        ])
                    
                    return objectives
        except:
            pass
        
        return ["No active mission", "Ready to accept new mission commands"]
    
    def get_recent_photos(self):
        """Get recent photos - placeholder for now"""
        return []  # TODO: Implement when mission provides photo paths

def main():
    app = MissionControlApp()
    
    # Sidebar for navigation
    st.sidebar.markdown('<div class="main-header" style="font-size: 1.5rem; margin-bottom: 1rem;">🎛️ MISSION CONTROL</div>', unsafe_allow_html=True)
    
    tab_selection = st.sidebar.radio(
        "**NAVIGATION**",
        ["🚁 Mission Planning", "📊 Mission Execution", "⚙️ System Status"]
    )
    
    # Connection status in sidebar
    st.sidebar.markdown("### 🔗 CONNECTION STATUS")
    st.sidebar.markdown('<div class="mission-panel">', unsafe_allow_html=True)
    
    # Check API server connection
    api_connected = check_api_connection()
    if api_connected:
        st.sidebar.markdown('<div style="color: #2ecc71;">🟢 **Mission API Server**<br/>CONNECTED</div>', unsafe_allow_html=True)
    else:
        st.sidebar.markdown('<div style="color: #e74c3c;">🔴 **Mission API Server**<br/>OFFLINE</div>', unsafe_allow_html=True)
        st.sidebar.markdown('<div class="mission-description">Start with:<br/>`python mission_api_server.py`</div>', unsafe_allow_html=True)
    
    # Check AirSim availability via API server
    api_connected = check_api_connection()
    if api_connected:
        # Test AirSim through API server
        try:
            airsim_success, airsim_result = test_airsim_connection()
            if airsim_success:
                st.sidebar.markdown('<div style="color: #2ecc71;">🟢 **AirSim**<br/>AVAILABLE</div>', unsafe_allow_html=True)
            else:
                st.sidebar.markdown('<div style="color: #f39c12;">🟡 **AirSim**<br/>NOT CONNECTED</div>', unsafe_allow_html=True)
                st.sidebar.markdown('<div class="mission-description">Click Play in UE5 to start simulation</div>', unsafe_allow_html=True)
        except:
            st.sidebar.markdown('<div style="color: #f39c12;">🟡 **AirSim**<br/>STATUS UNKNOWN</div>', unsafe_allow_html=True)
    else:
        st.sidebar.markdown('<div style="color: #e74c3c;">🔴 **AirSim**<br/>CANNOT CHECK</div>', unsafe_allow_html=True)
        st.sidebar.markdown('<div class="mission-description">API server offline</div>', unsafe_allow_html=True)
    
    st.sidebar.markdown('</div>', unsafe_allow_html=True)
    
    # Connection test buttons
    st.sidebar.markdown("### 🔧 DIAGNOSTICS")
    col_btn1, col_btn2 = st.sidebar.columns(2)
    
    with col_btn1:
        if st.button("🔧 TEST API", use_container_width=True):
            with st.sidebar:
                with st.spinner("Testing API..."):
                    if check_api_connection():
                        st.success("🟢 API CONNECTED")
                    else:
                        st.error("🔴 API OFFLINE")
    
    with col_btn2:
        if st.button("🚁 TEST AIRSIM", use_container_width=True):
            with st.sidebar:
                with st.spinner("Testing AirSim..."):
                    success, result = test_airsim_connection()
                    if success:
                        st.success("🟢 AIRSIM CONNECTED")
                        with st.expander("📊 AirSim Status"):
                            details = result.get('details', {})
                            st.write(f"**State:** {details.get('landed_state', 'Unknown')}")
                            pos = details.get('position', {})
                            st.write(f"**Position:** x={pos.get('x', 0)}, y={pos.get('y', 0)}, z={pos.get('z', 0)}")
                            st.write(f"**API Control:** {details.get('api_control', False)}")
                    else:
                        st.error("🔴 AIRSIM CONNECTION FAILED")
                        if isinstance(result, dict):
                            st.error(f"Error: {result.get('status', 'Unknown error')}")
                        else:
                            st.error(f"Error: {result}")
                        with st.expander("🔧 Troubleshooting"):
                            st.write("1. Make sure UE5 with AirSim is running")
                            st.write("2. Click Play in UE5 to start simulation")
                            st.write("3. Check AirSim settings.json exists")
                            st.write("4. Verify Mission API Server is running")
                            st.write("5. Check that API server is in AirSim environment")

    # Main content based on tab selection
    if tab_selection == "🚁 Mission Planning":
        app.render_mission_selection_tab()
    elif tab_selection == "📊 Mission Execution":
        app.render_mission_execution_tab()
    elif tab_selection == "⚙️ System Status":
        st.markdown('<div class="main-header">⚙️ SYSTEM STATUS</div>', unsafe_allow_html=True)
        
        # System info
        col1, col2 = st.columns(2)
        with col1:
            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
            st.markdown("**Python Version**")
            st.markdown(f"**{sys.version_info.major}.{sys.version_info.minor}**")
            st.markdown('</div>', unsafe_allow_html=True)
            
            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
            st.markdown("**Active Missions**")
            st.markdown(f"**{1 if st.session_state.mission_active else 0}**")
            st.markdown('</div>', unsafe_allow_html=True)
            
        with col2:
            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
            st.markdown("**Available Scripts**")
            st.markdown(f"**{len(app.load_mission_types())}**")
            st.markdown('</div>', unsafe_allow_html=True)
            
            st.markdown('<div class="metric-container">', unsafe_allow_html=True)
            st.markdown("**Log Entries**")
            st.markdown(f"**{len(st.session_state.mission_log)}**")
            st.markdown('</div>', unsafe_allow_html=True)
        
        # Mission logs
        st.markdown("### 📋 FULL MISSION LOG")
        st.markdown('<div class="config-panel">', unsafe_allow_html=True)
        st.markdown('<div class="mission-description" style="max-height: 400px; overflow-y: auto;">', unsafe_allow_html=True)
        if st.session_state.mission_log:
            for log_entry in reversed(st.session_state.mission_log):
                st.text(log_entry)
        else:
            st.text("No mission log entries yet")
        st.markdown('</div></div>', unsafe_allow_html=True)

if __name__ == "__main__":
    main() 