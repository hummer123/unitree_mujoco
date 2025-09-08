
#include <string>
#include <sstream>
#include <iostream>
#include <limits>

// Simple helper to parse command-like inputs of the form:
// <motor_id> <q> <dq> <kp> <kd> <tau>
// Example: "0 0.005 0.0 50.0 3.5 0.0"

struct MotorCommand {
	int motor_id = 0;
	double q = 0.0;
	double dq = 0.0;
	double kp = 0.0;
	double kd = 0.0;
	double tau = 0.0;
	bool valid = false;        // set true when parsing succeeded
	std::string error_msg;     // filled when parsing fails

	std::string toString() const {
		std::ostringstream oss;
		oss << "MotorCommand(motor_id=" << motor_id
			<< ", q=" << q
			<< ", dq=" << dq
			<< ", kp=" << kp
			<< ", kd=" << kd
			<< ", tau=" << tau << ")";
		return oss.str();
	}
};

class MotorCmdParse {
public:
	// Return a short help/usage string describing required and optional fields.
	static std::string PrintHelp() {
		std::ostringstream oss;
		oss << "\nUsage: <motor_id> <q> [dq] [kp] [kd] [tau]\n";
		oss << "  motor_id : integer (required)\n";
		oss << "  q        : position (required)\n";
		oss << "  dq       : velocity (optional, default=0.0)\n";
		oss << "  kp       : position gain (optional, default=0.0)\n";
		oss << "  kd       : velocity gain (optional, default=0.0)\n";
		oss << "  tau      : feedforward torque (optional, default=0.0)\n";
		oss << "Examples:\n";
		oss << "  \"0 0.005\"                      -- motor 0, q=0.005, others default\n";
		oss << "  \"1 -0.1 0.0 50.0 3.5 0.0\"  -- full six-field form\n";
		return oss.str();
	}

	// Parse a whitespace-separated string containing 6 fields in order:
	// motor_id q [dq kp kd tau]
	// motor_id and q are required; dq/kp/kd/tau are optional and default to 0.0.
	// Returns a MotorCommand with valid==true on success.
	static MotorCommand ParseFromString(const std::string &line) {
		MotorCommand out;
		std::istringstream iss(line);
		// Use long double for intermediate to get a bit more parsing stability
		long double q = 0.0L, dq = 0.0L, kp = 0.0L, kd = 0.0L, tau = 0.0L;
		int motor_id = 0;

		// motor_id (required)
		if (!(iss >> motor_id)) {
			out.valid = false;
			out.error_msg = "failed to parse motor_id (required)";
			return out;
		}

		// q (required)
		std::string tok;
		if (!(iss >> tok)) {
			out.valid = false;
			out.error_msg = "failed to parse q (required)";
			return out;
		}
		{
			std::istringstream ts(tok);
			if (!(ts >> q)) {
				out.valid = false;
				out.error_msg = "failed to parse q from token: '" + tok + "'";
				return out;
			}
		}

		// Optional params: dq, kp, kd, tau (in that order). If a token exists we parse it;
		// if it does not exist we leave the default 0.0.
		const char *names[] = {"dq", "kp", "kd", "tau"};
		long double *vals[] = {&dq, &kp, &kd, &tau};
		for (size_t i = 0; i < 4; ++i) {
			if (!(iss >> tok)) break; // no more tokens: fine
			std::istringstream ts(tok);
			long double v = 0.0L;
			if (!(ts >> v)) {
				out.valid = false;
				out.error_msg = std::string("failed to parse ") + names[i] + " from token: '" + tok + "'";
				return out;
			}
			*vals[i] = v;
		}

		// Ensure there's no extra token beyond the allowed six
		std::string extra;
		if (iss >> extra) {
			out.valid = false;
			out.error_msg = "extra token after expected fields: '" + extra + "'";
			return out;
		}

		out.motor_id = motor_id;
		out.q = static_cast<double>(q);
		out.dq = static_cast<double>(dq);
		out.kp = static_cast<double>(kp);
		out.kd = static_cast<double>(kd);
		out.tau = static_cast<double>(tau);
		out.valid = true;
		return out;
	}

	// Parse from traditional argc/argv. Expects at least program + motor_id + q
	// i.e. argc >= 3. Additional optional argv items are dq kp kd tau (up to 4).
	// Return valid==true on success. Does not modify program args.
	static MotorCommand ParseFromArgv(int argc, const char **argv) {
		MotorCommand out;
		if (argc < 3) {
			out.valid = false;
			out.error_msg = "expected at least 2 parameters: <motor_id> <q> (others optional: [dq] [kp] [kd] [tau])";
			return out;
		}

		std::ostringstream oss;
		// join all provided tokens (argv[1] .. argv[argc-1]) into a single string and reuse ParseFromString.
		for (int i = 1; i < argc; ++i) {
			if (i > 1) oss << ' ';
			oss << argv[i];
		}
		return ParseFromString(oss.str());
	}

	// Optional helper: clamp motor id into range [minId, maxId]. Useful before using the id.
	static void ClampMotorId(MotorCommand &cmd, int minId = 0, int maxId = 11) {
		if (!cmd.valid) return;
		if (cmd.motor_id < minId) cmd.motor_id = minId;
		if (cmd.motor_id > maxId) cmd.motor_id = maxId;
	}
};
