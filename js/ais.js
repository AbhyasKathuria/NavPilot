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
    this.vesselCategory = options.vesselCategory || (options.type === 'Patrol Craft' || options.type === 'Mini Ship' || options.type === 'Tug' ? 'mini' : 'cargo');
    this.name = options.name || (this.vesselCategory === 'mini' ? 'MINI CRAFT ' : 'CARGO SHIP ') + (this.mmsi % 1000);
    this.type = options.type || (this.vesselCategory === 'mini' ? 'Mini Ship' : 'Container Vessel');
    this.x = options.x; // grid coordinates (0 - 50)
    this.y = options.y;
    this.sog = options.sog !== undefined ? options.sog : (this.vesselCategory === 'mini' ? 16.0 : 12.0); // Speed Over Ground in knots
    this.cog = options.cog !== undefined ? options.cog : Math.random() * 360; // Course Over Ground in degrees
    this.heading = this.cog; // Heading in degrees

    if (this.vesselCategory === 'mini') {
      this.length = options.length || 65.0;
      this.beam = options.beam || 13.0;
      this.draft = options.draft || 4.5;
    } else {
      this.length = options.length || 190.0;
      this.beam = options.beam || 30.0;
      this.draft = options.draft || 11.5;
    }

    this.navStatus = this.sog <= 0.1 ? 'At anchor' : (options.navStatus || 'Underway using engine');

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

  isCellNavigable(gx, gy, grid) {
    if (!grid.inBounds(gx, gy)) return false;
    if (grid.cells[gy][gx] === grid.TYPE_LAND) return false;
    if (grid.depths[gy][gx] < this.draft * 0.85) return false;
    return true;
  }

  update(dt, grid) {
    if (this.sog <= 0.05) return;

    const prevX = this.x;
    const prevY = this.y;
    let nextX = prevX + this.vx * dt;
    let nextY = prevY + this.vy * dt;

    // Trail
    this.trail.push({ x: prevX, y: prevY });
    if (this.trail.length > this.maxTrail) {
      this.trail.shift();
    }

    const pad = 1.2;
    let bounced = false;

    if (nextX < pad) {
      nextX = pad;
      this.cog = (360 - this.cog) % 360;
      bounced = true;
    } else if (nextX > grid.cols - 1 - pad) {
      nextX = grid.cols - 1 - pad;
      this.cog = (360 - this.cog) % 360;
      bounced = true;
    }

    if (nextY < pad) {
      nextY = pad;
      this.cog = (180 - this.cog + 360) % 360;
      bounced = true;
    } else if (nextY > grid.rows - 1 - pad) {
      nextY = grid.rows - 1 - pad;
      this.cog = (180 - this.cog + 360) % 360;
      bounced = true;
    }

    // Lookahead shoreline check: never enter land or shallow reef above draft
    const lookX = nextX + Math.sign(this.vx) * 0.45;
    const lookY = nextY + Math.sign(this.vy) * 0.45;
    const gx = Math.round(nextX);
    const gy = Math.round(nextY);
    const lx = Math.round(lookX);
    const ly = Math.round(lookY);

    if (!this.isCellNavigable(gx, gy, grid) || !this.isCellNavigable(lx, ly, grid)) {
      // Revert to safe previous open water coordinate
      nextX = prevX;
      nextY = prevY;
      bounced = true;

      // Scan candidate deflection courses to find open sea
      const candidateOffsets = [90, -90, 135, -135, 180, 45, -45];
      let chosenCog = (this.cog + 180) % 360;
      for (let i = 0; i < candidateOffsets.length; i++) {
        const testCog = (this.cog + candidateOffsets[i] + 360) % 360;
        const rad = (testCog * Math.PI) / 180.0;
        const tx = Math.round(prevX + Math.sin(rad) * 1.2);
        const ty = Math.round(prevY - Math.cos(rad) * 1.2);
        if (this.isCellNavigable(tx, ty, grid)) {
          chosenCog = testCog;
          break;
        }
      }
      this.cog = chosenCog;
    }

    this.x = nextX;
    this.y = nextY;

    if (bounced) {
      this.heading = this.cog;
      this.updateVelocity();
    }
  }

  predictPosition(tSeconds, grid = null) {
    let px = this.x + this.vx * tSeconds;
    let py = this.y + this.vy * tSeconds;
    if (grid) {
      px = Math.max(1, Math.min(grid.cols - 2, px));
      py = Math.max(1, Math.min(grid.rows - 2, py));
    }
    return { x: px, y: py };
  }

  /**
   * AIS safety domain envelope (radius in meters)
   */
  get domainRadiusMeters() {
    if (this.vesselCategory === 'mini') {
      return Math.max(24.0, this.length * 0.65);
    }
    return Math.max(38.0, this.length * 0.65);
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

  /**
   * Places a user-defined Cargo Ship or Mini Ship onto the chart.
   */
  addCustomVessel(options) {
    const category = options.vesselCategory || 'cargo';
    const count = this.vessels.length + 1;
    const defaultName = category === 'mini'
      ? `MINI CRAFT ${String(count).padStart(2, '0')}`
      : `CARGO VESSEL ${String(count).padStart(2, '0')}`;
    const defaultType = category === 'mini' ? 'Mini Ship' : 'Container Vessel';

    return this.addVessel({
      name: options.name || defaultName,
      type: options.type || defaultType,
      vesselCategory: category,
      x: options.x,
      y: options.y,
      sog: options.sog !== undefined ? options.sog : (category === 'mini' ? 16.0 : 12.0),
      cog: options.cog !== undefined ? options.cog : 0,
      length: options.length || (category === 'mini' ? 65 : 195),
      beam: options.beam || (category === 'mini' ? 13 : 32),
      draft: options.draft || (category === 'mini' ? 4.5 : 11.5)
    });
  }

  removeVessel(mmsi) {
    const idx = this.vessels.findIndex(v => String(v.mmsi) === String(mmsi));
    if (idx !== -1) {
      return this.vessels.splice(idx, 1)[0];
    }
    return null;
  }

  removeNearest(x, y, maxDistCells = 2.2) {
    let bestIdx = -1;
    let bestDist = maxDistCells;
    for (let i = 0; i < this.vessels.length; i++) {
      const d = Math.hypot(this.vessels[i].x - x, this.vessels[i].y - y);
      if (d <= bestDist) {
        bestDist = d;
        bestIdx = i;
      }
    }
    if (bestIdx !== -1) {
      return this.vessels.splice(bestIdx, 1)[0];
    }
    return null;
  }

  addRandomVessel(category = null) {
    for (let attempts = 0; attempts < 60; attempts++) {
      const x = 6 + Math.random() * (this.grid.cols - 12);
      const y = 6 + Math.random() * (this.grid.rows - 12);
      const gx = Math.round(x);
      const gy = Math.round(y);

      if (this.grid.inBounds(gx, gy) && this.grid.cells[gy][gx] === this.grid.TYPE_SEA) {
        const chosenCat = category || (Math.random() < 0.35 ? 'mini' : 'cargo');
        const cargoTypes = ['Container Vessel', 'Oil Tanker', 'Bulk Carrier'];
        const miniTypes = ['Mini Ship', 'Patrol Craft', 'Harbor Tug'];
        const chosenType = chosenCat === 'mini'
          ? miniTypes[Math.floor(Math.random() * miniTypes.length)]
          : cargoTypes[Math.floor(Math.random() * cargoTypes.length)];
        const names = chosenCat === 'mini'
          ? ['COASTAL TUG', 'HARBOR PILOT', 'SWIFT PATROL', 'MINI FEEDER']
          : ['OCEAN LEADER', 'NORDIC TRADER', 'GLOBAL CHALLENGER', 'MARITIME VOYAGER', 'ORIENT EXPRESS'];
        const chosenName = names[Math.floor(Math.random() * names.length)] + ' ' + Math.floor(Math.random() * 90 + 10);

        return this.addVessel({
          name: chosenName,
          type: chosenType,
          vesselCategory: chosenCat,
          x,
          y,
          sog: chosenCat === 'mini' ? 14.0 + Math.random() * 8.0 : 9.0 + Math.random() * 7.0,
          cog: Math.floor(Math.random() * 360),
          length: chosenCat === 'mini' ? 50 + Math.random() * 35 : 175 + Math.random() * 90,
          beam: chosenCat === 'mini' ? 10 + Math.random() * 6 : 28 + Math.random() * 10,
          draft: chosenCat === 'mini' ? 4.5 : 10 + Math.random() * 4
        });
      }
    }
    return null;
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
   * Spatiotemporal conflict check for planners (A*, RRT*, Monkey Search).
   * Predicts AIS vessel positions at arrival time tSeconds (and immediate t <= 2s window).
   */
  isSpatiotemporalSafe(x, y, tSeconds = 0, shipDomainMeters = 48.0, extraMarginMeters = 22.0) {
    for (let i = 0; i < this.vessels.length; i++) {
      const v = this.vessels[i];
      const combinedBuffer = shipDomainMeters + v.domainRadiusMeters + extraMarginMeters;

      // Always avoid moored/stationary vessels (sog <= 0.2) or current occupied hull zone if t is near 0
      const vPos = v.sog <= 0.2
        ? { x: v.x, y: v.y }
        : v.predictPosition(Math.min(tSeconds, 45.0), this.grid);

      const dMeters = Math.hypot(x - vPos.x, y - vPos.y) * this.grid.cellSizeMeters;
      if (dMeters < combinedBuffer) {
        return false;
      }

      // Also check near-present position if transit starts quickly (t < 8s)
      if (tSeconds < 8.0) {
        const dNowMeters = Math.hypot(x - v.x, y - v.y) * this.grid.cellSizeMeters;
        if (dNowMeters < (shipDomainMeters + v.domainRadiusMeters * 0.85)) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * Checks a line segment (x0,y0) -> (x1,y1) over arrival interval [t0, t1] against all AIS vessels.
   */
  isSegmentSpatiotemporalSafe(x0, y0, x1, y1, t0, t1, shipDomainMeters = 48.0) {
    const dist = Math.hypot(x1 - x0, y1 - y0);
    const steps = Math.max(2, Math.ceil(dist / 0.5));
    for (let s = 0; s <= steps; s++) {
      const u = s / steps;
      const x = x0 + (x1 - x0) * u;
      const y = y0 + (y1 - y0) * u;
      const t = t0 + (t1 - t0) * u;
      if (!this.isSpatiotemporalSafe(x, y, t, shipDomainMeters, 15.0)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Evaluates CPA, TCPA, and collision risk of each AIS vessel
   * against the ASV ship and its domain SS = L * B * D.
   * Evaluates starting from t = 0.0 so zero close-range encounters are ever skipped!
   */
  evaluateTrafficRisk(asv, horizonSeconds = 30.0) {
    const asvSpeedCells = (asv.speed > 0.5 ? asv.speed : asv.maxSpeedMs) / this.grid.cellSizeMeters;
    const asvDomainRadius = asv.domainRadiusMeters;

    let highestRiskVessel = null;
    let minCpaMeters = Infinity;
    let minTcpaSeconds = Infinity;
    let conflictDetected = false;
    let emergencyImminent = false;

    for (let i = 0; i < this.vessels.length; i++) {
      const v = this.vessels[i];
      const combinedSafetyBuffer = asvDomainRadius + v.domainRadiusMeters + 12.0;

      let closestDistMeters = Infinity;
      let closestTime = Infinity;

      const dtSample = 0.4;
      const steps = Math.ceil(horizonSeconds / dtSample);

      for (let s = 0; s <= steps; s++) {
        const t = s * dtSample;

        // ASV predicted position along its active path
        const distAlongCells = asvSpeedCells * t;
        const asvPos = (t === 0)
          ? { x: asv.x, y: asv.y }
          : this.getFutureAsvPosition(asv, distAlongCells);
        if (!asvPos) break;

        // AIS Vessel predicted position
        const vPos = v.predictPosition(t, this.grid);

        const dCells = Math.hypot(asvPos.x - vPos.x, asvPos.y - vPos.y);
        const dMeters = dCells * this.grid.cellSizeMeters;

        if (dMeters < closestDistMeters) {
          closestDistMeters = dMeters;
          closestTime = t;
        }
      }

      // Also check instantaneous hull separation
      const currentDistMeters = Math.hypot(asv.x - v.x, asv.y - v.y) * this.grid.cellSizeMeters;
      const physicalHullMinMeters = (asv.beam + v.beam) * 0.5 + 22.0;

      v.cpaMeters = closestDistMeters;
      v.tcpaSeconds = closestTime;

      if (currentDistMeters < physicalHullMinMeters * 1.4) {
        emergencyImminent = true;
      }

      // Risk classification
      if ((closestDistMeters < combinedSafetyBuffer && closestTime < horizonSeconds) ||
          currentDistMeters < combinedSafetyBuffer) {
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
      emergencyImminent,
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
