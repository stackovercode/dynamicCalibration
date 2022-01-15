package com.spin.dynamicCalibration.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.domain.URCapAPI;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.data.DataModel;

public class ServerShutdownProgramNodeContribution implements ProgramNodeContribution {
	private final URCapAPI urCapAPI;
	private final DataModel dataModel;

	public ServerShutdownProgramNodeContribution(URCapAPI urCapAPI, DataModel dataModel) {
		this.urCapAPI = urCapAPI;
		this.dataModel = dataModel;
	}

	@Override
	public void openView() {}

	@Override
	public void closeView() {}

	@Override
	public String getTitle() {return "Client shutdown";}

	@Override
	public boolean isDefined() {return true;}

	@Override
	public void generateScript(ScriptWriter writer) {
		// Closing server and connection:
		writer.appendLine("socketStatus = False");
		writer.appendLine("socket_close()");
	}
}