var Vector = require("/Users/decisiontoolbox/dev/steering/vector");//.


/**
 * Represents a vehicle
 * @constructor
 * @param {Vector} pos - The vehicle's position.
 * @param {Vector} vel - The vehicle's velocity.
 */
var Vehicle = function(pos, vel) {
	this.position = pos || new Vector;
	this.velocity = vel || new Vector;
}


//-- Static Properties --\\
//.this is not what I want. figure out better place to config.
if(true) {
	MAX_SPEED = 20;
	MAX_FORCE = 5;
	MASS = 1;
	PERCEPTION = 30;
}
Vehicle.prototype.maxSpeed = MAX_SPEED;		// The vehicle's max speed.
Vehicle.prototype.maxForce = MAX_FORCE;		// The vehicle's max force.
Vehicle.prototype.mass = MASS;				// The vehicle's mass, which affects acceleration.
Vehicle.prototype.perception = PERCEPTION;	// How far the vehicle can "see."


/**
 * Physics-based locomotion. This takes a force vector and updates the vehicle's velocity and position.
 * @param {Vector} force Steering force to be applied to the vehicle.
 * @param {Number} dt Change in time or "delta time" used for applying force gradually or multiple times per second.
 * @return {Vector} The vehicle's actual acceleration after applying the modified force (current velocity minus previous velocity).
 */
Vehicle.prototype.applyForce = function(force, dt) {
	dt = Number(dt || 1);

	force = force.limit(this.maxForce); // Don't let the vehicle apply more force than it's able to.
	//.should we save acceleration as an instance variable? might need to if we want to apply external forces
	var acceleration = force.div(this.mass); // Newton's 2nd law: F=ma (or a=F/m)

	var prevVelocity = new Vector(this.velocity.x, this.velocity.y);

	// Add acceleration to velocity and velocity (modified by dt) to position.
	this.velocity = this.velocity.add(acceleration).limit(this.maxSpeed); // Don't let the vehicle move faster than it's able to.
	this.position = this.position.add(this.velocity.scale(dt));

	return this.velocity.sub(prevVelocity);
};



//-- Steering Behaviors --\\

/**
 * THE BIG ONE
 * steering = desired - current
 * @param {Vector} desired The vehicle's desired velocity.
 */
Vehicle.prototype.steer = function(desired) {
	desired = desired.limit(this.maxSpeed);		// 1. Don't let the vehicle desire the impossible.
	var steering = desired.sub(this.velocity);	// 2. Calculate the difference between the vehicle's current and desired velocities.
	return steering.limit(this.maxForce);		// 3. Limit that steering force to the vehicle's max force.
};


/**
 * Go AFAP toward the target.
 * @param {Vector} target The point that the vehicle is trying to reach.
 */
Vehicle.prototype.seek = function(target) {
	var desired = target.sub(this.position).setMagnitude(this.maxSpeed);
	return this.steer(desired);
};

/**
 * Go AFAP away from the target.
 * @param target Vector The point that the vehicle is fleeing from.
 */
Vehicle.prototype.flee = function(target) {
	return this.seek(target).mul(-1);
};

/**
 * Go AFAP toward the target until nearby, then approach slowly.
 * @param {Vector} target The point that the vehicle is trying to reach.
 */
Vehicle.prototype.arrive = function(target) {
	var radius = this.maxSpeed; // "slowing radius" - if farther, seek AFAP

	var desired = target.sub(this.position); // desired direction
	var sqrD = desired.sqrMag();

	if(sqrD < radius*radius) {
		// If we're close, approach slowly.
		return this.steer(desired.setMagnitude(Math.sqrt(sqrD) * this.maxSpeed / radius));
	} else {
		// Else, seek AFAP!
		return this.seek(target);
	}
};

/**
 * Anticipate where the target will be and seek its future position.
 * @param {Vehicle} target The object that this vehicle is trying to catch.
 */
Vehicle.prototype.pursue = function(target) {
	return this.seek(target.position.add(target.velocity));
};

/**
 * Anticipate where the target will be and flee from its future position.
 * @param {Vehicle} target The object that this vehicle is evading.
 */
Vehicle.prototype.evade = function(target) {
	return this.flee(target.position.add(target.velocity));
};

/**
 * Gradually slow the vehicle down by steering in the direction opposite its current velocity.
 */
Vehicle.prototype.brake = function() {
	return this.steer(this.velocity.mul(-1));
};

/**
 * Move away from vehicles that are too close.
 * @param {[Vehicle]} neighbors A list of neighboring vehicles to separate from.
 */
Vehicle.prototype.separate = function(neighbors) {
	var v = new Vector;
	if(neighbors.length == 0) return v;

	var sqrLeeway = this.velocity.sqrMag() / this.maxForce; // less space is needed at lower speeds

	var count = 0;

	neighbors.forEach(function(neighbor) {
		var sqrD = this.position.sqrDist(neighbor.position);

		// If it's too close for comfort...
		if(sqrD < sqrLeeway) {
			// ...find the vector pointing away from the neighbor...
			var away = this.position.sub(neighbor.position);
			// ...and weight that vector by distance (smaller distance => greater repulsion).
			v = v.add(away.setMagnitude(1 / Math.sqrt(sqrD)));
			count++;
		}
	}, this);

	if(count === 0) return v;

	// Steer away AFAP.
	return this.steer(v.setMagnitude(this.maxSpeed));
};

/**
 * Find the average direction and steer in that direction.
 * @param {[Vehicle]} neighbors A list of neighboring vehicles to align with.
 */
Vehicle.prototype.align = function(neighbors) {
	var v = new Vector;
	if(neighbors.length == 0) return v;

	neighbors.forEach(function(neighbor){
		// Add the neighbor's current velocity.
		v = v.add(neighbor.velocity);
	}, this);

	// Steer in the neighbors' average direction AFAP.
	return this.steer(v.setMagnitude(this.maxSpeed));
};

/**
 * Arrive at the center of mass.
 * @param {[Vehicle]} neighbors A list of neighboring vehicles to cohere with.
 */
Vehicle.prototype.cohere = function(neighbors) {
	var v = new Vector;
	if(neighbors.length == 0) return v;

	neighbors.forEach(function(neighbor){
		// Add the neighbor's current position.
		v = v.add(neighbor.position);
	}, this);

	return this.arrive(v.div(neighbors.length));
};

//.https://www.red3d.com/cwr/steer/gdc99/
//.avoid obstacles
//.avoid collisions
//.wander
//.flow field
//.offset pursuit
//.shadow (similar to offset pursuit?)
//.follow leader
//.interpose (get between two other vehicles)



/**
 * Find vehicles close enough to be called a neighbor.
 * @param vehicles [Vehicle] A list of vehicles that could possibly be neighbors.
 */
Vehicle.prototype.neighbors = function(vehicles) {
	var neighbors = [];
	var sqrPerception = Math.pow(this.perception, 2);

	vehicles.forEach(function(vehicle){
		if(vehicle == this) return;

		// Is it inside the perception circle's outer square?
		if(vehicle.position.x > this.position.x - this.perception &&
			vehicle.position.x < this.position.x + this.perception &&
			vehicle.position.y > this.position.y - this.perception &&
			vehicle.position.y < this.position.y + this.perception)
		{
			// Is it inside the actual perception radius?
			if(vehicle.position.sqrDist(this.position) < sqrPerception) {
				neighbors.push(vehicle);
			}
		}
	}, this);

	return neighbors;
};



// more complex behaviors

/**
 * Flocking is a composite steering behavior made up of 3 other behaviors:
 * @see separate
 * @see align
 * @see cohere
 * @param {[Vehicle]} vehicles The vehicles in the flock.
 * @param {Number} sep Relative amount to weight the separation force.
 * @param {Number} ali Relative amount to weight the alignment force.
 * @param {Number} coh Relative amount to weight the cohesion force.
 */
Vehicle.prototype.flock = function(vehicles, sep, ali, coh) {
	var separation = this.separate(vehicles).scale(sep);
	var alignment = this.align(vehicles).scale(ali);
	var cohesion = this.cohere(vehicles).scale(coh);

	return separation.add(alignment).add(cohesion).limit(this.maxForce);
};



module.exports = Vehicle;
