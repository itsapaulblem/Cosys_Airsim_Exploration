#!/usr/bin/env python3
"""
MAVLink Remote Client Registration API
Provides REST API for dynamic registration of external MAVLink clients
Enables runtime addition/removal of external IP endpoints
"""

import asyncio
import json
import logging
import os
import subprocess
import time
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any
from pathlib import Path

try:
    from fastapi import FastAPI, HTTPException, BackgroundTasks, Depends
    from fastapi.security import HTTPBasic, HTTPBasicCredentials
    from fastapi.middleware.cors import CORSMiddleware
    from pydantic import BaseModel, validator
    import uvicorn
    import secrets
except ImportError:
    print("❌ Required packages not installed. Install with:")
    print("   pip install fastapi uvicorn python-multipart")
    exit(1)

# Logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Security
security = HTTPBasic()

class ClientRegistration(BaseModel):
    """External client registration model"""
    ip: str
    port: int
    client_type: str = "qgc"  # qgc, api, custom
    description: str = ""
    heartbeat_interval: int = 30  # seconds
    timeout: int = 300  # seconds
    
    @validator('ip')
    def validate_ip(cls, v):
        import ipaddress
        try:
            ipaddress.ip_address(v)
            return v
        except ValueError:
            raise ValueError('Invalid IP address')
            
    @validator('port')
    def validate_port(cls, v):
        if not 1 <= v <= 65535:
            raise ValueError('Port must be between 1 and 65535')
        return v
        
    @validator('client_type')
    def validate_client_type(cls, v):
        if v not in ['qgc', 'api', 'custom']:
            raise ValueError('client_type must be qgc, api, or custom')
        return v

class ClientStatus(BaseModel):
    """Client status model"""
    id: str
    ip: str
    port: int
    client_type: str
    description: str
    registered_at: datetime
    last_heartbeat: Optional[datetime]
    status: str  # active, inactive, timeout
    connection_count: int

class MAVLinkRegistrationAPI:
    """MAVLink client registration and management API"""
    
    def __init__(self, config_dir: str = "/px4_workspace/mavlink-router-config"):
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
        self.clients: Dict[str, Dict] = {}
        self.client_file = self.config_dir / "registered_clients.json"
        self.config_file = self.config_dir / "dynamic_config.conf"
        
        # Load existing clients
        self.load_clients()
        
        # Configuration
        self.max_clients = int(os.getenv('MAX_CLIENTS', '10'))
        self.default_timeout = int(os.getenv('CLIENT_TIMEOUT', '300'))
        self.config_reload_command = os.getenv('CONFIG_RELOAD_CMD', 'nginx -s reload')
        
    def load_clients(self) -> None:
        """Load existing client registrations"""
        try:
            if self.client_file.exists():
                with open(self.client_file, 'r') as f:
                    data = json.load(f)
                    self.clients = data.get('clients', {})
                logger.info(f"Loaded {len(self.clients)} existing clients")
        except Exception as e:
            logger.error(f"Error loading clients: {e}")
            self.clients = {}
            
    def save_clients(self) -> None:
        """Save client registrations to file"""
        try:
            data = {
                'clients': self.clients,
                'last_updated': datetime.now().isoformat()
            }
            with open(self.client_file, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            logger.info(f"Saved {len(self.clients)} clients to {self.client_file}")
        except Exception as e:
            logger.error(f"Error saving clients: {e}")
            
    def generate_client_id(self, ip: str, port: int) -> str:
        """Generate unique client ID"""
        return f"{ip}_{port}_{int(time.time())}"
        
    def register_client(self, registration: ClientRegistration) -> str:
        """Register new external client"""
        
        # Check client limit
        if len(self.clients) >= self.max_clients:
            raise HTTPException(status_code=429, detail="Maximum client limit reached")
            
        # Check for existing client with same IP:port
        existing = self.find_client_by_endpoint(registration.ip, registration.port)
        if existing:
            raise HTTPException(status_code=409, detail="Client already registered")
            
        # Generate client ID
        client_id = self.generate_client_id(registration.ip, registration.port)
        
        # Create client record
        client_data = {
            'id': client_id,
            'ip': registration.ip,
            'port': registration.port,
            'client_type': registration.client_type,
            'description': registration.description or f"{registration.client_type.upper()} client",
            'registered_at': datetime.now(),
            'last_heartbeat': datetime.now(),
            'heartbeat_interval': registration.heartbeat_interval,
            'timeout': registration.timeout,
            'status': 'active',
            'connection_count': 0
        }
        
        # Add to clients
        self.clients[client_id] = client_data
        
        # Save and regenerate configuration
        self.save_clients()
        self.regenerate_mavlink_config()
        
        logger.info(f"Registered client {client_id}: {registration.ip}:{registration.port}")
        return client_id
        
    def unregister_client(self, client_id: str) -> bool:
        """Unregister external client"""
        
        if client_id not in self.clients:
            return False
            
        client_data = self.clients.pop(client_id)
        self.save_clients()
        self.regenerate_mavlink_config()
        
        logger.info(f"Unregistered client {client_id}: {client_data['ip']}:{client_data['port']}")
        return True
        
    def find_client_by_endpoint(self, ip: str, port: int) -> Optional[str]:
        """Find client by IP and port"""
        for client_id, client_data in self.clients.items():
            if client_data['ip'] == ip and client_data['port'] == port:
                return client_id
        return None
        
    def update_client_heartbeat(self, client_id: str) -> bool:
        """Update client heartbeat timestamp"""
        
        if client_id not in self.clients:
            return False
            
        self.clients[client_id]['last_heartbeat'] = datetime.now()
        self.clients[client_id]['status'] = 'active'
        self.clients[client_id]['connection_count'] += 1
        
        # Save periodically (not every heartbeat to avoid I/O overhead)
        if self.clients[client_id]['connection_count'] % 10 == 0:
            self.save_clients()
            
        return True
        
    def get_client_status(self, client_id: str) -> Optional[Dict]:
        """Get client status"""
        
        if client_id not in self.clients:
            return None
            
        client_data = self.clients[client_id].copy()
        
        # Update status based on last heartbeat
        if client_data['last_heartbeat']:
            last_heartbeat = client_data['last_heartbeat']
            if isinstance(last_heartbeat, str):
                last_heartbeat = datetime.fromisoformat(last_heartbeat)
                
            timeout_threshold = datetime.now() - timedelta(seconds=client_data['timeout'])
            if last_heartbeat < timeout_threshold:
                client_data['status'] = 'timeout'
            else:
                client_data['status'] = 'active'
                
        return client_data
        
    def get_all_clients(self) -> List[Dict]:
        """Get all registered clients with status"""
        
        clients = []
        for client_id in self.clients:
            client_status = self.get_client_status(client_id)
            if client_status:
                clients.append(client_status)
                
        return clients
        
    def cleanup_expired_clients(self) -> int:
        """Remove expired clients"""
        
        expired_clients = []
        current_time = datetime.now()
        
        for client_id, client_data in self.clients.items():
            last_heartbeat = client_data.get('last_heartbeat')
            if isinstance(last_heartbeat, str):
                last_heartbeat = datetime.fromisoformat(last_heartbeat)
                
            if last_heartbeat:
                timeout_threshold = current_time - timedelta(seconds=client_data['timeout'])
                if last_heartbeat < timeout_threshold:
                    expired_clients.append(client_id)
                    
        # Remove expired clients
        for client_id in expired_clients:
            self.unregister_client(client_id)
            
        if expired_clients:
            logger.info(f"Cleaned up {len(expired_clients)} expired clients")
            
        return len(expired_clients)
        
    def regenerate_mavlink_config(self) -> bool:
        """Regenerate MAVLink Router configuration with registered clients"""
        
        try:
            # Create external clients configuration
            external_clients = []
            for client_data in self.clients.values():
                if client_data['status'] == 'active':
                    external_clients.append({
                        'ip': client_data['ip'],
                        'port': client_data['port'],
                        'type': client_data['client_type'],
                        'description': client_data['description']
                    })
                    
            # Save external clients file for config generator
            clients_config = {'clients': external_clients}
            clients_file = self.config_dir / "external_clients.json"
            
            with open(clients_file, 'w') as f:
                json.dump(clients_config, f, indent=2)
                
            # Generate enhanced MAVLink Router configuration
            script_path = Path(__file__).parent.parent / "scripts" / "generate_mavlink_config_enhanced.py"
            template_path = Path(__file__).parent.parent / "scripts" / "mavlink-router-enhanced-template.conf"
            output_path = self.config_dir / "enhanced-router.conf"
            
            cmd = [
                'python3', str(script_path),
                '--template', str(template_path),
                '--output', str(output_path),
                '--external-clients', str(clients_file),
                '--validate'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info("MAVLink Router configuration regenerated successfully")
                
                # Trigger configuration reload (if configured)
                if self.config_reload_command:
                    try:
                        subprocess.run(self.config_reload_command.split(), check=True)
                        logger.info("Configuration reloaded successfully")
                    except subprocess.CalledProcessError as e:
                        logger.warning(f"Configuration reload failed: {e}")
                        
                return True
            else:
                logger.error(f"Configuration generation failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error regenerating configuration: {e}")
            return False

# Initialize API
registration_api = MAVLinkRegistrationAPI()
app = FastAPI(
    title="MAVLink Registration API",
    description="Dynamic registration API for external MAVLink clients",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Authentication
def authenticate(credentials: HTTPBasicCredentials = Depends(security)):
    """Simple HTTP Basic authentication"""
    correct_username = secrets.compare_digest(credentials.username, os.getenv('API_USERNAME', 'admin'))
    correct_password = secrets.compare_digest(credentials.password, os.getenv('API_PASSWORD', 'admin123'))
    
    if not (correct_username and correct_password):
        raise HTTPException(
            status_code=401,
            detail="Invalid credentials",
            headers={"WWW-Authenticate": "Basic"},
        )
    return credentials.username

# API Endpoints

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "registered_clients": len(registration_api.clients),
        "api_version": "1.0.0"
    }

@app.post("/api/v1/clients/register")
async def register_client(
    registration: ClientRegistration,
    background_tasks: BackgroundTasks,
    username: str = Depends(authenticate)
):
    """Register a new external MAVLink client"""
    
    try:
        client_id = registration_api.register_client(registration)
        
        # Schedule configuration regeneration in background
        background_tasks.add_task(registration_api.regenerate_mavlink_config)
        
        return {
            "client_id": client_id,
            "status": "registered",
            "message": f"Client {registration.ip}:{registration.port} registered successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Registration error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.delete("/api/v1/clients/{client_id}")
async def unregister_client(
    client_id: str,
    background_tasks: BackgroundTasks,
    username: str = Depends(authenticate)
):
    """Unregister an external MAVLink client"""
    
    if registration_api.unregister_client(client_id):
        background_tasks.add_task(registration_api.regenerate_mavlink_config)
        return {"status": "unregistered", "client_id": client_id}
    else:
        raise HTTPException(status_code=404, detail="Client not found")

@app.post("/api/v1/clients/{client_id}/heartbeat")
async def client_heartbeat(client_id: str):
    """Update client heartbeat (no auth required for heartbeats)"""
    
    if registration_api.update_client_heartbeat(client_id):
        return {"status": "heartbeat_updated", "timestamp": datetime.now().isoformat()}
    else:
        raise HTTPException(status_code=404, detail="Client not found")

@app.get("/api/v1/clients/{client_id}")
async def get_client_status(
    client_id: str,
    username: str = Depends(authenticate)
):
    """Get client status"""
    
    client_status = registration_api.get_client_status(client_id)
    if client_status:
        return client_status
    else:
        raise HTTPException(status_code=404, detail="Client not found")

@app.get("/api/v1/clients")
async def list_clients(username: str = Depends(authenticate)):
    """List all registered clients"""
    
    clients = registration_api.get_all_clients()
    return {
        "clients": clients,
        "total_count": len(clients),
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/v1/maintenance/cleanup")
async def cleanup_expired_clients(
    background_tasks: BackgroundTasks,
    username: str = Depends(authenticate)
):
    """Clean up expired clients"""
    
    cleaned_count = registration_api.cleanup_expired_clients()
    
    if cleaned_count > 0:
        background_tasks.add_task(registration_api.regenerate_mavlink_config)
        
    return {
        "status": "cleanup_completed",
        "expired_clients_removed": cleaned_count,
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/v1/maintenance/reload-config")
async def reload_mavlink_config(username: str = Depends(authenticate)):
    """Manually trigger MAVLink configuration reload"""
    
    success = registration_api.regenerate_mavlink_config()
    
    if success:
        return {"status": "config_reloaded", "timestamp": datetime.now().isoformat()}
    else:
        raise HTTPException(status_code=500, detail="Configuration reload failed")

@app.get("/api/v1/stats")
async def get_statistics(username: str = Depends(authenticate)):
    """Get API statistics"""
    
    clients = registration_api.get_all_clients()
    
    stats = {
        "total_clients": len(clients),
        "active_clients": len([c for c in clients if c['status'] == 'active']),
        "inactive_clients": len([c for c in clients if c['status'] == 'timeout']),
        "client_types": {},
        "timestamp": datetime.now().isoformat()
    }
    
    # Count by client type
    for client in clients:
        client_type = client['client_type']
        stats['client_types'][client_type] = stats['client_types'].get(client_type, 0) + 1
        
    return stats

# Background task for cleanup
async def periodic_cleanup():
    """Periodic cleanup of expired clients"""
    while True:
        try:
            registration_api.cleanup_expired_clients()
            await asyncio.sleep(60)  # Run every minute
        except Exception as e:
            logger.error(f"Periodic cleanup error: {e}")
            await asyncio.sleep(60)

@app.on_event("startup")
async def startup_event():
    """Start background tasks"""
    asyncio.create_task(periodic_cleanup())
    logger.info("MAVLink Registration API started")

if __name__ == "__main__":
    # Configuration
    host = os.getenv('API_HOST', '0.0.0.0')
    port = int(os.getenv('API_PORT', '8000'))
    log_level = os.getenv('LOG_LEVEL', 'info')
    
    # Run API server
    uvicorn.run(
        "mavlink-registration-api:app",
        host=host,
        port=port,
        log_level=log_level,
        reload=False
    )