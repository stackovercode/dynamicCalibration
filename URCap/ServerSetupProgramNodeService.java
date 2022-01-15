package com.spin.dynamicCalibration.impl;

import java.util.Locale;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.*;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;
import com.ur.urcap.api.domain.data.DataModel;

public class ServerSetupProgramNodeService implements SwingProgramNodeService<ServerSetupProgramNodeContribution, ServerSetupProgramNodeView> {
	public ServerSetupProgramNodeService() {}

	@Override
	public String getId() {return "ClientSetupProgramNode";}

	@Override
	public void configureContribution(ContributionConfiguration configuration) {configuration.setChildrenAllowed(false);}

	@Override
	public String getTitle(Locale locale) {return "Client Setup";}

		@Override
	public ServerSetupProgramNodeView createView(ViewAPIProvider apiProvider) {
		return new ServerSetupProgramNodeView(apiProvider);
	}

	@Override
	public ServerSetupProgramNodeContribution createNode(ProgramAPIProvider apiProvider,
			ServerSetupProgramNodeView view, DataModel model, CreationContext context) {
		return new ServerSetupProgramNodeContribution(apiProvider, view, model);
	}
}