//-- Vehicle class --\\

function Vehicle(opts) {
	if(typeof opts === "undefined") opts = {};

	this.position = opts.position || new Vector;
	this.velocity = opts.velocity || new Vector;
}

// static properties
Vehicle.prototype.max_speed = MAX_SPEED;	// The vehicle's max speed.
Vehicle.prototype.max_force = MAX_FORCE;	// The vehicle's max force.
Vehicle.prototype.mass = MASS;				// The vehicle's mass, which affects acceleration.
Vehicle.prototype.perception = PERCEPTION;	// How far the vehicle can "see."
Vehicle.prototype.leeway = LEEWAY;			// Elbow room required by the vehicle when separating.

Vehicle.prototype.apply_force = function(force, dt) {
	dt = dt || 1;//.(unit) test this

	force = force.limit(this.max_force);
	var acc = force.scale(1 / this.mass);

	// Add acceleration to velocity and velocity to position.
	this.velocity = this.velocity.add(acc).limit(this.max_speed);
	this.position = this.position.add(this.velocity.scale(dt));
}



//-- Steering Behaviors --\\

/**
 * steering = desired - current
 * @param desired Vector The vehicle's desired velocity.
 */
Vehicle.prototype.steer = function(desired) {
	desired = desired.limit(this.max_speed);	// 1. Don't let the vehicle desire the impossible.
	var steering = desired.sub(this.velocity);	// 2. Calculate the difference between the vehicle's current and desired velocities.
	return steering.limit(this.max_force);		// 3. Limit that steering force to the vehicle's max force.
}

/**
 * Go AFAP toward the target.
 * @param target Vector The point that the vehicle is trying to reach.
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
	return -this.seek(target);//.test this
}

/**
 * Go AFAP toward the target until nearby, then approach slowly.
 * @param target Vector The point that the vehicle is trying to reach.
 */
Vehicle.prototype.arrive = function(target) {
	var desired = target.sub(this.position);
	var sq_d = desired.sqrMag();//.test these changes

	var arbitrary = 50 * 50;//.
	if(sq_d > arbitrary) {
		return this.seek(desired);
	} else {
		return this.steer(desired.setMagnitude(Math.sqrt(sq_d) * this.max_speed / arbitrary));
	}
}

/**
 * Anticipate where the target will be and seek its future position.
 * @param target Vehicle The object that this vehicle is trying to catch.
 */
Vehicle.prototype.pursue = function(target) {
	return this.seek(target.position + target.velocity);
}

/**
 * Anticipate where the target will be and flee from its future position.
 * @param target Vehicle The object that this vehicle is trying to catch.
 */
Vehicle.prototype.evade = function(target) {
	return this.flee(target.position + target.velocity);
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
