#!/usr/bin/env python3
"""
Fleet Database Reset Script

Safely clears stuck jobs and resets drone status without breaking the system.
This preserves the database schema and drone registry.

Use this when:
- Drones are stuck BUSY with old jobs
- Jobs are piling up in QUEUED state
- System was stopped/crashed mid-mission
"""

import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path


def reset_fleet_database():
    """Reset fleet database to clean state"""

    db_path = Path(__file__).parent / "fleet_dispatch" / "fleet.db"

    if not db_path.exists():
        print(f"❌ Database not found at: {db_path}")
        print("   Make sure you're in the DroneProgram directory")
        return False

    print(f"📁 Database location: {db_path}")
    print("⚠️  This will:")
    print("   - Delete all jobs (QUEUED, ASSIGNED, IN_PROGRESS, COMPLETED)")
    print("   - Reset all drones to AVAILABLE status")
    print("   - Clear current_job_id from all drones")
    print("   - Preserve drone registry and schema")
    print()

    response = input("Continue? (yes/no): ").strip().lower()
    if response not in ["yes", "y"]:
        print("❌ Cancelled")
        return False

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Count current data
        cursor.execute("SELECT COUNT(*) FROM jobs")
        job_count = cursor.fetchone()[0]

        cursor.execute("SELECT COUNT(*) FROM drones WHERE status = 'BUSY'")
        busy_count = cursor.fetchone()[0]

        print("\n📊 Current state:")
        print(f"   - Total jobs: {job_count}")
        print(f"   - Busy drones: {busy_count}")

        # Step 1: Delete all jobs
        print("\n🗑️  Deleting all jobs...")
        cursor.execute("DELETE FROM jobs")
        deleted_jobs = cursor.rowcount
        print(f"   ✅ Deleted {deleted_jobs} jobs")

        # Step 2: Reset all drones to AVAILABLE
        # Timezone-aware: naive datetimes cannot be compared against the
        # aware ones every other writer produces, and utcnow() is deprecated.
        now = datetime.now(timezone.utc).isoformat()
        print("\n🔄 Resetting drones to AVAILABLE...")
        cursor.execute(
            """
            UPDATE drones
            SET status = 'AVAILABLE',
                current_job_id = NULL,
                last_heartbeat = ?
            WHERE status != 'AVAILABLE'
        """,
            (now,),
        )
        reset_drones = cursor.rowcount
        print(f"   ✅ Reset {reset_drones} drones to AVAILABLE")

        # Step 3: Verify final state
        cursor.execute("SELECT id, status, current_job_id FROM drones")
        drones = cursor.fetchall()

        print("\n✅ Final state:")
        for drone_id, status, job_id in drones:
            print(f"   - {drone_id}: {status} (job: {job_id})")

        # Commit changes
        conn.commit()
        conn.close()

        print("\n🎉 Database reset complete!")
        print("\n📝 Next steps:")
        print("   1. Restart fleet_dispatch: cd fleet_dispatch && uvicorn app:app --port 5000")
        print("   2. Check status: curl http://127.0.0.1:5000/fleet/status | jq")
        print("   3. Dispatch a new test job from the UI")
        print("\n⚠️  Note: Make sure PX4 SITL is running before dispatching jobs!")

        return True

    except sqlite3.Error as e:
        print(f"\n❌ Database error: {e}")
        return False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False


if __name__ == "__main__":
    print("=" * 70)
    print(" Fleet Database Reset Tool")
    print("=" * 70)
    print()

    success = reset_fleet_database()
    sys.exit(0 if success else 1)
