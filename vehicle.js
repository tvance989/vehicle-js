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
//.maybe make Vehicle abstract and force proper config when concretizing?
if(true) {
	MAX_SPEED = 20;
	MAX_FORCE = 5;
	MASS = 1;
	PERCEPTION = 30;
	LEEWAY = 5;
}
Vehicle.prototype.max_speed = MAX_SPEED;	// The vehicle's max speed.
Vehicle.prototype.max_force = MAX_FORCE;	// The vehicle's max force.
Vehicle.prototype.mass = MASS;				// The vehicle's mass, which affects acceleration.
Vehicle.prototype.perception = PERCEPTION;	// How far the vehicle can "see."
Vehicle.prototype.leeway = LEEWAY;			// Elbow room required by the vehicle when separating.


/**
 * @param {Number} dt Multiplier for position.//.word this better
 */
Vehicle.prototype.apply_force = function(force, dt) {
	dt = dt || 1;

	force = force.limit(this.max_force);
	var acc = force.scale(1 / this.mass);

	// Add acceleration to velocity and velocity to position.
	this.velocity = this.velocity.add(acc).limit(this.max_speed);
	this.position = this.position.add(this.velocity.scale(dt));
}



//-- Steering Behaviors --\\

/**
 * THE BIG ONE
 * steering = desired - current
 * @param {Vector} desired The vehicle's desired velocity.
 */
Vehicle.prototype.steer = function(desired) {
	desired = desired.limit(this.max_speed);	// 1. Don't let the vehicle desire the impossible.
	var steering = desired.sub(this.velocity);	// 2. Calculate the difference between the vehicle's current and desired velocities.
	return steering.limit(this.max_force);		// 3. Limit that steering force to the vehicle's max force.
}

/**
 * Go AFAP toward the target.
 * @param {Vector} target The point that the vehicle is trying to reach.
 */
Vehicle.prototype.seek = function(target) {
	var desired = target.sub(this.position).setMagnitude(this.max_speed);
	return this.steer(desired);
}

/**
 * Go AFAP away from the target.
 * @param target Vector The point that the vehicle is fleeing from.
 */
Vehicle.prototype.flee = function(target) {
	return this.seek(target).mul(-1);
}

/**
 * Go AFAP toward the target until nearby, then approach slowly.
 * @param {Vector} target The point that the vehicle is trying to reach.
 * @param {Number} radius Slowing radius. If closer than this, approach slowly. Else, seek AFAP. Defaults to vehicle's max speed / max force.
 */
Vehicle.prototype.arrive = function(target, radius) {
	radius = radius || (this.max_speed / this.max_force);

	var desired = target.sub(this.position); // desired direction
	var sq_d = desired.sqrMag();

	if(sq_d < radius*radius) {
		// If we're close, approach slowly.
		return this.steer(desired.setMagnitude(Math.sqrt(sq_d) * this.max_speed / radius));
	} else {
		// Else, seek AFAP!
		return this.seek(target);
	}
}

/**
 * Anticipate where the target will be and seek its future position.
 * @param {Vehicle} target The object that this vehicle is trying to catch.
 */
Vehicle.prototype.pursue = function(target) {
	return this.seek(target.position.add(target.velocity));
}

/**
 * Anticipate where the target will be and flee from its future position.
 * @param target Vehicle The object that this vehicle is trying to catch.
 */
Vehicle.prototype.evade = function(target) {
	return this.flee(target.position.add(target.velocity));
}

/**
 * Move away from vehicles that are too close.
 * @param neighbors [Vehicle] A list of neighboring vehicles to separate from.
 */
Vehicle.prototype.separate = function(neighbors) {
	var v = new Vector;
	var count = 0;

	neighbors.forEach(function(neighbor){
		var d = this.position.distance(neighbor.position);
		// If it's too close for comfort...
		if(d < this.leeway) {
			// ...find the vector pointing away from the neighbor...
			var away = this.position.sub(neighbor.position);
			// ...and weight that vector by distance (smaller distance => greater repulsion).
			v = v.add(away.setMagnitude(1 / d));
			count++;
		}
	}, this);

	if(count == 0) return v;

	// Steer away AFAP (as fast as possible).
	return this.steer(v.setMagnitude(this.max_speed));
}

/**
 * Find the average direction and steer in that direction.
 * @param neighbors [Vehicle] A list of neighboring vehicles to align with.
 */
Vehicle.prototype.align = function(neighbors) {
	var v = new Vector;
	if(neighbors.length == 0) return v;

	neighbors.forEach(function(neighbor){
		// Add the neighbor's current velocity.
		v = v.add(neighbor.velocity);
	}, this);

	// Steer in the neighbors' average direction AFAP.
	return this.steer(v.setMagnitude(this.max_speed));
}

/**
 * Arrive at the center of mass.
 * @param neighbors [Vehicle] A list of neighboring vehicles to cohere with.
 */
Vehicle.prototype.cohere = function(neighbors) {
	var v = new Vector;
	if(neighbors.length == 0) return v;

	neighbors.forEach(function(neighbor){
		// Add the neighbor's current position.
		v = v.add(neighbor.position);
	}, this);

	v = v.scale(1 / neighbors.length);
	return this.arrive(v);
}






//.not sure the best place for this stuff...

/**
 * Find vehicles close enough to be called a neighbor.
 * @param vehicles [Vehicle] A list of vehicles that could be neighbors.
 */
Vehicle.prototype.neighbors = function(vehicles) {
	var neighbors = [];
	var sqrD = Math.pow(this.perception, 2);

	vehicles.forEach(function(vehicle){
		if(vehicle == this) return;

		// Is it inside the perception circle's outer square?
		if(vehicle.position.x > this.position.x - this.perception &&
			vehicle.position.x < this.position.x + this.perception &&
			vehicle.position.y > this.position.y - this.perception &&
			vehicle.position.y < this.position.y + this.perception)
		{
			// Is it inside the actual perception radius?
			if(vehicle.position.sqrDist(this.position) < sqrD) {
				neighbors.push(vehicle);
			}
		}
	}, this);

	return neighbors;
}

/*
Vehicle.prototype.flock = function() {
	var neighbors = this.neighbors();

	var separation = this.separate(neighbors).scale(SEPARATION_WEIGHT);
	var alignment = this.align(neighbors).scale(ALIGNMENT_WEIGHT);
	var cohesion = this.cohere(neighbors).scale(COHESION_WEIGHT);

	return separation.add(alignment).add(cohesion);
}
*/



module.exports = Vehicle;
