-- Obstacle Memory Database Schema
-- Stores persistent obstacle information with confidence decay

CREATE TABLE IF NOT EXISTS obstacles (
    id TEXT PRIMARY KEY,
    lat REAL NOT NULL,
    lon REAL NOT NULL,
    radius_m REAL NOT NULL DEFAULT 3.0,
    obstacle_type TEXT DEFAULT 'static_wall',
    confidence REAL NOT NULL DEFAULT 0.5,
    source_drone TEXT,
    first_seen TEXT NOT NULL,
    last_confirmed TEXT NOT NULL
);

-- Index for bounding box queries
CREATE INDEX IF NOT EXISTS idx_obstacles_location ON obstacles(lat, lon);

-- Index for source drone queries
CREATE INDEX IF NOT EXISTS idx_obstacles_source ON obstacles(source_drone);
