package com.spin.dynamicCalibration.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.ProgramNodeService;
import com.ur.urcap.api.domain.URCapAPI;
import com.ur.urcap.api.domain.data.DataModel;
import java.io.InputStream;

public class ServerShutdownProgramNodeService implements ProgramNodeService {
	public ServerShutdownProgramNodeService() {}

	@Override
	public String getId() {return "ClientShutdownProgramNode";}

	@Override
	public String getTitle() {return "Client shutdown";}

	@Override
	public InputStream getHTML() {return this.getClass().getResourceAsStream("resources/html/ServerShutdownProgramNode.html");	}

	@Override
	public boolean isDeprecated() {return false;}

	@Override
	public boolean isChildrenAllowed() {return false;}

	@Override
	public ProgramNodeContribution createNode(URCapAPI urCapAPI, DataModel dataModel) {return new ServerShutdownProgramNodeContribution(urCapAPI, dataModel);}
}