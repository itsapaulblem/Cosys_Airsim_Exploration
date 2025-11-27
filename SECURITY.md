# Security Policy

## Overview

This document outlines security practices and requirements for the AirSim ROS2 Multi-Robot Architecture project. Proper security configuration is critical for protecting sensitive data and maintaining system integrity.

## 🔒 Critical Security Requirements

### Environment Variable Management

**NEVER commit hardcoded secrets to version control.** All sensitive configuration must be provided via environment variables.

#### Required Environment Variables

| Variable | Purpose | Required For |
|----------|---------|-------------|
| `POSTGRES_PASSWORD` | PostgreSQL database password | Database services |
| `PGADMIN_PASSWORD` | pgAdmin web interface password | Database administration |
| `AIRSIM_DB_PASSWORD` | Alternative PostgreSQL password | Legacy compatibility |

#### Optional Security Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `POSTGRES_USER` | Database username | `airsim_user` |
| `POSTGRES_DB` | Database name | `drone_statistics` |
| `PGADMIN_EMAIL` | pgAdmin login email | `admin@example.com` |

## 🛡️ Secure Setup Guide

### Step 1: Environment Configuration

1. **Copy the environment templates:**
   ```bash
   # Root configuration
   cp .env.template .env

   # Docker configuration
   cp docker/.env.example docker/.env
   ```

2. **Generate secure passwords:**
   ```bash
   # Method 1: OpenSSL (32 characters)
   openssl rand -base64 32

   # Method 2: Python (URL-safe, 32 characters)
   python -c "import secrets; print(secrets.token_urlsafe(32))"

   # Method 3: System entropy (Linux/macOS)
   head -c 32 /dev/urandom | base64
   ```

3. **Update environment files:**
   - Replace `generate_a_secure_password_here` with actual secure passwords
   - Ensure passwords are 20+ characters with mixed case, numbers, and symbols
   - Use different passwords for each environment (dev/staging/prod)

### Step 2: Validation

Before starting services, verify:

```bash
# Check environment variables are set
echo "POSTGRES_PASSWORD: ${POSTGRES_PASSWORD:-(not set)}"
echo "PGADMIN_PASSWORD: ${PGADMIN_PASSWORD:-(not set)}"

# Verify .env files exist but are not committed
ls -la .env docker/.env
git status | grep -E "\.env$" || echo "✓ .env files not in git"
```

### Step 3: Service Startup

```bash
# Test PostgreSQL connection (will fail if password not set)
docker-compose -f docker/docker-compose-master.yml --profile stats-only up postgres-stats

# Start full ecosystem with secure configuration
docker-compose -f docker/docker-compose-master.yml --profile integrated up
```

## 🚨 Security Incidents

### Recently Resolved Issues

**2024-09-18**: Removed hardcoded secrets from codebase
- ✅ Fixed: PostgreSQL password in `database_interface.py`
- ✅ Fixed: Docker compose password defaults
- ✅ Fixed: Android File Server security token
- ✅ Added: Environment variable validation
- ✅ Created: Secure configuration templates

### Historical Vulnerabilities

1. **Hardcoded Database Passwords** (Fixed)
   - Files: `ros2/src/airsim_ros_pkgs/scripts/database_interface.py`
   - Impact: Database access exposure
   - Resolution: Environment variable requirement with validation

2. **Docker Default Passwords** (Fixed)
   - Files: `docker/docker-compose-master.yml`
   - Impact: Default admin access
   - Resolution: Removed fallback values, requires explicit configuration

3. **Security Token Exposure** (Fixed)
   - Files: `Unreal/Environments/Blocks/Config/DefaultEngine.ini`
   - Impact: Android File Server unauthorized access
   - Resolution: Generated new cryptographically secure token

## 🔍 Security Validation Checklist

### Pre-Deployment Security Audit

- [ ] No hardcoded secrets in source code
- [ ] All sensitive data provided via environment variables
- [ ] Environment files (.env) in .gitignore
- [ ] Secure passwords (20+ characters, complexity requirements)
- [ ] Different passwords for each environment
- [ ] Database connections encrypted in production
- [ ] Regular password rotation schedule established

### Development Security

- [ ] Pre-commit hooks for secret detection installed
- [ ] Code review includes security validation
- [ ] Development databases use non-production credentials
- [ ] Local .env files never committed to repository

### Production Security

- [ ] Secrets management system implemented (Azure Key Vault, AWS Secrets Manager, etc.)
- [ ] Database connections use TLS/SSL
- [ ] Regular security vulnerability scans
- [ ] Access logging and monitoring enabled
- [ ] Backup encryption enabled

## 🔧 Android File Server Token Configuration

### Setup Process

The Android File Server requires a secure token for operation. This token is environment-specific and should never be committed to git.

1. **Generate a secure token:**
   ```bash
   # Method 1: Python (recommended)
   python -c "import secrets; print(secrets.token_hex(16).upper())"

   # Method 2: OpenSSL
   openssl rand -hex 16 | tr '[:lower:]' '[:upper:]'

   # Method 3: PowerShell (Windows)
   -join ((1..32) | ForEach {'{0:X}' -f (Get-Random -Max 16)})
   ```

2. **Configure your environment:**
   ```bash
   # Copy the template
   cp Unreal/Environments/Blocks/Config/DefaultEngine.ini.template \
      Unreal/Environments/Blocks/Config/DefaultEngine.ini

   # Edit the file and replace REPLACE_WITH_SECURE_TOKEN with your generated token
   # Example: SecurityToken=A1B2C3D4E5F6789012345678ABCDEF90
   ```

3. **Verify security:**
   - ✅ DefaultEngine.ini is in .gitignore (never committed)
   - ✅ Each environment uses a unique token
   - ✅ Token is 32 characters of uppercase hex
   - ✅ Template file contains placeholder only

### Token Rotation

Rotate Android File Server tokens regularly:
- **Development**: Every 90 days
- **Production**: Every 30 days
- **After security incidents**: Immediately

## 🔧 Tooling and Automation

### Pre-commit Hooks

Install secret detection:

```bash
# Install pre-commit
pip install pre-commit

# Install git-secrets
git clone https://github.com/awslabs/git-secrets.git
cd git-secrets && make install

# Configure for this repository
git secrets --register-aws
git secrets --install
git secrets --scan
```

### CI/CD Security

Add to your pipeline:

```yaml
# Example GitHub Actions security check
- name: Scan for secrets
  uses: trufflesecurity/trufflehog@main
  with:
    path: ./
    base: main
    head: HEAD
```

## 📞 Reporting Security Issues

If you discover a security vulnerability:

1. **DO NOT** create a public issue
2. Send details to the project maintainers privately
3. Include:
   - Description of the vulnerability
   - Steps to reproduce
   - Potential impact assessment
   - Suggested remediation (if any)

## 🔄 Security Maintenance

### Regular Tasks

- **Weekly**: Review access logs for anomalies
- **Monthly**: Update dependencies and security patches
- **Quarterly**: Rotate database passwords
- **Annually**: Full security audit and penetration testing

### Monitoring

Key security indicators to monitor:
- Failed database connection attempts
- Unusual API access patterns
- Large data exports
- Administrative access outside business hours

## 📚 Additional Resources

- [OWASP Secrets Management Guide](https://owasp.org/www-community/vulnerabilities/Use_of_hard-coded_password)
- [Docker Secrets Management](https://docs.docker.com/engine/swarm/secrets/)
- [PostgreSQL Security Checklist](https://www.postgresql.org/docs/current/security-checklist.html)
- [ROS2 Security Guidelines](https://docs.ros.org/en/galactic/Tutorials/Advanced/Security/)

---

**Last Updated**: September 18, 2024
**Security Version**: 1.0
**Next Review**: December 18, 2024