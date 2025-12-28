# Migration Guide: Local Qdrant to Qdrant Cloud

This guide documents the migration from local Docker Qdrant to Qdrant Cloud and provides rollback procedures.

## Overview

The backend has been refactored to use Qdrant Cloud instead of local Docker Qdrant. This document covers:

- Migration procedures
- Rollback procedures
- Troubleshooting

## Pre-Migration Checklist

Before migrating, ensure you have:

- [ ] Qdrant Cloud account created
- [ ] New cluster provisioned
- [ ] API key generated
- [ ] Current data exported (if needed)
- [ ] All environment variables documented

## Migration Steps

### 1. Backup Existing Data (Optional)

If you have existing data in local Qdrant:

```bash
# Export collection data using qdrant-client
python -c "
from qdrant_client import QdrantClient
client = QdrantClient(host='localhost', port=6333)
points, _ = client.scroll(collection_name='textbook_content', limit=10000)
print(f'Found {len(points)} points to migrate')
# Save to file for re-import if needed
import json
with open('backup.json', 'w') as f:
    json.dump([{'id': p.id, 'vector': p.vector, 'payload': p.payload} for p in points], f)
"
```

### 2. Configure Qdrant Cloud

Update your `.env` file:

```bash
# Old local configuration (remove)
# QDRANT_HOST=localhost
# QDRANT_PORT=6333

# New cloud configuration (add)
QDRANT_CLUSTER_ENDPOINT=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-cloud-api-key
QDRANT_TIMEOUT=10.0
```

### 3. Validate Configuration

```bash
python scripts/validate_env.py --qdrant-only
```

Expected output:
```
=== Qdrant Cloud Configuration ===
✓ QDRANT_CLUSTER_ENDPOINT configured
✓ QDRANT_API_KEY configured
✓ Qdrant Cloud connection successful
Collections: X
```

### 4. Index Content to Cloud

```bash
# Re-index all textbook content to Qdrant Cloud
python scripts/index_content.py --source ../book-source/docs
```

This will:
- Connect to Qdrant Cloud
- Create the `textbook_content` collection if needed
- Index all markdown files
- Report statistics

### 5. Update Application Configuration

Remove any local Qdrant references from:

- `docker-compose.yml` (if used)
- Environment variables
- Documentation

### 6. Restart Application

```bash
# Restart with new configuration
uvicorn main:app --reload
```

### 7. Verify Functionality

```bash
# Test health endpoint
curl http://localhost:8000/api/v1/chat/health

# Test chat functionality
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"content": "What is ROS 2?"}'
```

## Rollback Procedure

If you need to revert to local Qdrant:

### Option 1: Revert Environment Variables

```bash
# Edit .env file
# Comment out cloud configuration
# QDRANT_CLUSTER_ENDPOINT=https://your-cluster.qdrant.io
# QDRANT_API_KEY=your-cloud-api-key

# Uncomment local configuration
QDRANT_HOST=localhost
QDRANT_PORT=6333
```

### Option 2: Use Docker Compose (if available)

```bash
# Start local Qdrant
docker-compose up -d qdrant

# Wait for Qdrant to be ready
sleep 5

# Restore data from backup (if needed)
python -c "
import json
from qdrant_client import QdrantClient, models

with open('backup.json') as f:
    data = json.load(f)

client = QdrantClient(host='localhost', port=6333)
client.upsert(
    collection_name='textbook_content',
    points=[
        models.PointStruct(id=p['id'], vector=p['vector'], payload=p['payload'])
        for p in data
    ]
)
print(f'Restored {len(data)} points')
"
```

### Option 3: Temporary Configuration Switch

```bash
# Run with environment variables inline
QDRANT_HOST=localhost QDRANT_PORT=6333 uvicorn main:app --reload
```

## Troubleshooting

### Connection Refused

**Error**: `Connection refused` or `Failed to connect to Qdrant Cloud`

**Solution**:
1. Verify cluster is running in Qdrant Cloud dashboard
2. Check API key is correct (no extra spaces)
3. Ensure URL starts with `https://`
4. Check network connectivity

### Authentication Failed (401)

**Error**: `401 Unauthorized` or `Authentication failed`

**Solution**:
1. Regenerate API key in Qdrant Cloud dashboard
2. Update `QDRANT_API_KEY` in `.env`
3. Ensure no special characters in key are being escaped

### Collection Not Found

**Error**: `Collection not found` or `404`

**Solution**:
1. Run indexing script to create collection:
   ```bash
   python scripts/index_content.py --source ../book-source/docs
   ```
2. Verify collection name matches (`textbook_content`)

### Timeout Errors

**Error**: `Timeout` or `Connection timeout`

**Solution**:
1. Increase timeout in `.env`:
   ```env
   QDRANT_TIMEOUT=30.0
   ```
2. Check network latency to Qdrant Cloud
3. Consider Qdrant Cloud region closer to your location

## Data Consistency

### Verifying Data Migration

```bash
# Check point count in Qdrant Cloud
python -c "
from qdrant_client import QdrantClient
from src.config.env import get_environment_config

config = get_environment_config()
client = QdrantClient(url=config.qdrant_cluster_endpoint, api_key=config.qdrant_api_key)
points, _ = client.scroll(collection_name='textbook_content', limit=1)
print(f'Collection has points')
"
```

### Comparing Local vs Cloud

```bash
# Count local points
docker exec qdrant qdrant-cli info localhost:6333 | grep points

# Count cloud points (from above Python script)
```

## Configuration Reference

### Environment Variables

| Variable | Local Default | Cloud Required |
|----------|--------------|----------------|
| `QDRANT_CLUSTER_ENDPOINT` | N/A | Yes (URL) |
| `QDRANT_API_KEY` | N/A | Yes (key) |
| `QDRANT_HOST` | localhost | N/A |
| `QDRANT_PORT` | 6333 | N/A |
| `QDRANT_TIMEOUT` | 10.0 | 10.0 |

### Collection Configuration

**Vector Schema** (must match):
- Size: 384
- Distance: COSINE
- Collection name: `textbook_content`

## Support

For issues:
1. Check Qdrant Cloud status page
2. Review application logs with `LOG_LEVEL=DEBUG`
3. Validate environment: `python scripts/validate_env.py`
