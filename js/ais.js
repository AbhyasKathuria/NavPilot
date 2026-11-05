/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * AIS Data Simulation System (ais.js)
 *
 * Implements real-time Automatic Identification System (AIS) marine traffic:
 * - Broadcasts MMSI, Vessel Name, Vessel Type, SOG, COG, Heading, Length, Beam, Draft
 * - Real-time dynamic vessel simulation (moves before, during, and after path planning)
 * - Trajectory prediction and Closest Point of Approach (CPA / TCPA) calculation
 * - Dynamic collision risk assessment against the ASV Ship Domain (SS = L * B * D)
 */

class AISVessel {
  constructor(options) {
    this.mmsi = options.mmsi || Math.floor(200000000 + Math.random() * 700000000);
    this.name = options.name || 'VESSEL ' + (this.mmsi % 1000);
    this.type = options.type || 'Container Vessel'; // 'Container Vessel', 'Oil Tanker', 'Bulk Carrier', 'Tug', 'Patrol Craft'
    this.x = options.x; // grid coordinates (0 - 50)
    this.y = options.y;
    this.sog = options.sog || 12.0; // Speed Over Ground in knots
    this.cog = options.cog !== undefined ? options.cog : Math.random() * 360; // Course Over Ground in degrees
    this.heading = this.cog; // Heading in degrees
    this.length = options.length || 180.0; // meters
    this.beam = options.beam || 28.0;     // meters
    this.draft = options.draft || 10.5;   // meters
    this.navStatus = options.navStatus || 'Underway using engine';

    // Velocity in grid cells per second (1 cell = 50m, 1 knot = 0.514444 m/s)
    this.updateVelocity();

    // Trail history for wake rendering
    this.trail = [];
    this.maxTrail = 40;

    // Computed CPA/TCPA relative to own ASV
    this.cpaMeters = Infinity;
    this.tcpaSeconds = Infinity;
    this.riskLevel = 'CLEAR'; // 'CLEAR', 'CAUTION', 'WARNING', 'COLLISION_ALERT'
  }

  updateVelocity() {
    const speedMs = this.sog * 0.514444;
    const speedCellsPerSec = speedMs / 50.0;
    const rad = (this.cog * Math.PI) / 180.0;
    this.vx = Math.sin(rad) * speedCellsPerSec;
    this.vy = -Math.cos(rad) * speedCellsPerSec; // grid 0,0 is top-left, North is -Y
  }

  update(dt, grid) {
    this.x += this.vx * dt;
    this.y += this.vy * dt;

    // Trail
    this.trail.push({ x: this.x, y: this.y });
    if (this.trail.length > this.maxTrail) {
      this.trail.shift();
    }

    // Boundary bounce / wrap with course reversal
    const pad = 1.0;
    let bounced = false;

    if (this.x < pad) {
      this.x = pad;
      this.cog = 360 - this.cog;
      bounced = true;
    } else if (this.x > grid.cols - 1 - pad) {
      this.x = grid.cols - 1 - pad;
      this.cog = 360 - this.cog;
      bounced = true;
    }

    if (this.y < pad) {
      this.y = pad;
      this.cog = (180 - this.cog + 360) % 360;
      bounced = true;
    } else if (this.y > grid.rows - 1 - pad) {
      this.y = grid.rows - 1 - pad;
      this.cog = (180 - this.cog + 360) % 360;
      bounced = true;
    }

    // Avoid running aground on static land
    const gx = Math.round(this.x);
    const gy = Math.round(this.y);
    if (grid.inBounds(gx, gy) && grid.cells[gy][gx] === grid.TYPE_LAND) {
      this.cog = (this.cog + 140 + Math.random() * 80) % 360;
      bounced = true;
    }

    if (bounced) {
      this.heading = this.cog;
      this.updateVelocity();
    }
  }

  predictPosition(tSeconds) {
    return {
      x: this.x + this.vx * tSeconds,
      y: this.y + this.vy * tSeconds
    };
  }

  /**
   * AIS safety domain envelope (radius in meters)
   */
  get domainRadiusMeters() {
    return Math.max(35.0, this.length * 0.65);
  }
}

class AISSystem {
  constructor(grid) {
    this.grid = grid;
    this.vessels = [];
    this.initDefaultTraffic();
  }

  initDefaultTraffic() {
    this.vessels = [];

    // Vessel 1: Large Container Vessel in main shipping lane
    this.vessels.push(new AISVessel({
      mmsi: 563042000,
      name: 'MV PACIFIC PIONEER',
      type: 'Container Vessel',
      x: 16.0,
      y: 10.0,
      sog: 14.5,
      cog: 135,
      length: 260,
      beam: 38,
      draft: 13.5
    }));

    // Vessel 2: Oil Tanker crossing southern approach
    this.vessels.push(new AISVessel({
      mmsi: 211345000,
      name: 'STENA POLARIS',
      type: 'Oil Tanker',
      x: 38.0,
      y: 34.0,
      sog: 11.2,
      cog: 295,
      length: 220,
      beam: 36,
      draft: 14.0
    }));

    // Vessel 3: Bulk Carrier navigating east-west channel
    this.vessels.push(new AISVessel({
      mmsi: 636019000,
      name: 'COSCO SHIPPING ASIA',
      type: 'Bulk Carrier',
      x: 29.0,
      y: 18.0,
      sog: 12.0,
      cog: 210,
      length: 195,
      beam: 32,
      draft: 11.8
    }));

    // Vessel 4: Fast Patrol Craft / Pilot Vessel
    this.vessels.push(new AISVessel({
      mmsi: 477123000,
      name: 'COASTAL PATROL 08',
      type: 'Patrol Craft',
      x: 10.0,
      y: 36.0,
      sog: 18.5,
      cog: 50,
      length: 85,
      beam: 16,
      draft: 6.2
    }));
  }

  addVessel(options) {
    const vessel = new AISVessel(options);
    this.vessels.push(vessel);
    return vessel;
  }

  addRandomVessel() {
    for (let attempts = 0; attempts < 50; attempts++) {
      const x = 6 + Math.random() * (this.grid.cols - 12);
      const y = 6 + Math.random() * (this.grid.rows - 12);
      const gx = Math.round(x);
      const gy = Math.round(y);

      if (this.grid.inBounds(gx, gy) && this.grid.cells[gy][gx] === this.grid.TYPE_SEA) {
        const types = ['Container Vessel', 'Oil Tanker', 'Bulk Carrier', 'Patrol Craft'];
        const chosenType = types[Math.floor(Math.random() * types.length)];
        const names = ['OCEAN LEADER', 'NORDIC TRADER', 'GLOBAL CHALLENGER', 'MARITIME VOYAGER', 'ORIENT EXPRESS'];
        const chosenName = names[Math.floor(Math.random() * names.length)] + ' ' + Math.floor(Math.random() * 90 + 10);

        return this.addVessel({
          name: chosenName,
          type: chosenType,
          x,
          y,
          sog: 9.0 + Math.random() * 8.0,
          cog: Math.floor(Math.random() * 360),
          length: chosenType === 'Patrol Craft' ? 80 : 180 + Math.random() * 100,
          beam: chosenType === 'Patrol Craft' ? 16 : 28 + Math.random() * 12,
          draft: chosenType === 'Patrol Craft' ? 5.5 : 10 + Math.random() * 4
        });
      }
    }
  }

  clear() {
    this.vessels = [];
  }

  update(dt) {
    for (let i = 0; i < this.vessels.length; i++) {
      this.vessels[i].update(dt, this.grid);
    }
  }

  /**
   * Evaluates CPA, TCPA, and collision risk of each AIS vessel
   * against the ASV ship and its domain SS = L * B * D.
   */
  evaluateTrafficRisk(asv, horizonSeconds = 30.0) {
    const asvSpeedCells = (asv.speed > 0.5 ? asv.speed : asv.maxSpeedMs) / this.grid.cellSizeMeters;
    const asvDomainRadius = asv.domainRadiusMeters;

    let highestRiskVessel = null;
    let minCpaMeters = Infinity;
    let minTcpaSeconds = Infinity;
    let conflictDetected = false;

    for (let i = 0; i < this.vessels.length; i++) {
      const v = this.vessels[i];
      const combinedSafetyBuffer = asvDomainRadius + v.domainRadiusMeters;

      let closestDistMeters = Infinity;
      let closestTime = Infinity;

      // Sample along forward time horizon
      const dtSample = 0.5;
      const steps = Math.ceil(horizonSeconds / dtSample);

      for (let s = 1; s <= steps; s++) {
        const t = s * dtSample;

        // ASV predicted position along its active path
        const distAlongCells = asvSpeedCells * t;
        const asvPos = this.getFutureAsvPosition(asv, distAlongCells);
        if (!asvPos) break;

        // AIS Vessel predicted position
        const vPos = v.predictPosition(t);

        const dCells = Math.hypot(asvPos.x - vPos.x, asvPos.y - vPos.y);
        const dMeters = dCells * this.grid.cellSizeMeters;

        if (dMeters < closestDistMeters) {
          closestDistMeters = dMeters;
          closestTime = t;
        }
      }

      v.cpaMeters = closestDistMeters;
      v.tcpaSeconds = closestTime;

      // Risk classification
      if (closestDistMeters < combinedSafetyBuffer && closestTime < horizonSeconds) {
        v.riskLevel = 'COLLISION_ALERT';
        conflictDetected = true;
        if (closestDistMeters < minCpaMeters) {
          minCpaMeters = closestDistMeters;
          minTcpaSeconds = closestTime;
          highestRiskVessel = v;
        }
      } else if (closestDistMeters < combinedSafetyBuffer * 1.6 && closestTime < horizonSeconds) {
        v.riskLevel = 'WARNING';
      } else if (closestDistMeters < combinedSafetyBuffer * 2.5) {
        v.riskLevel = 'CAUTION';
      } else {
        v.riskLevel = 'CLEAR';
      }
    }

    return {
      conflictDetected,
      criticalVessel: highestRiskVessel,
      cpaMeters: minCpaMeters,
      tcpaSeconds: minTcpaSeconds
    };
  }

  getFutureAsvPosition(asv, distCells) {
    if (!asv.activePath || asv.pathIndex >= asv.activePath.length) return null;

    let accumulatedDist = 0;
    let lastX = asv.x;
    let lastY = asv.y;

    for (let i = asv.pathIndex; i < asv.activePath.length; i++) {
      const pt = asv.activePath[i];
      const d = Math.hypot(pt.x - lastX, pt.y - lastY);
      if (accumulatedDist + d >= distCells) {
        const remaining = distCells - accumulatedDist;
        const ratio = d > 0 ? remaining / d : 0;
        return {
          x: lastX + (pt.x - lastX) * ratio,
          y: lastY + (pt.y - lastY) * ratio
        };
      }
      accumulatedDist += d;
      lastX = pt.x;
      lastY = pt.y;
    }

    return asv.activePath[asv.activePath.length - 1];
  }
}
