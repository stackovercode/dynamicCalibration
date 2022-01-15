package com.spin.dynamicCalibration.impl;

import java.util.Locale;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.*;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;
import com.ur.urcap.api.domain.data.DataModel;

public class SendCMDProgramNodeService implements SwingProgramNodeService<SendCMDProgramNodeContribution, SendCMDProgramNodeView> {
	@Override
	public String getId() {return "SendCMDProgramNode";}

	@Override
	public void configureContribution(ContributionConfiguration configuration) {configuration.setChildrenAllowed(false);}

	@Override
	public String getTitle(Locale locale) {return "Send a command";}

	@Override
	public SendCMDProgramNodeView createView(ViewAPIProvider apiProvider) {
		return new SendCMDProgramNodeView(apiProvider);
	}

	@Override
	public SendCMDProgramNodeContribution createNode(ProgramAPIProvider apiProvider,
			SendCMDProgramNodeView view, DataModel model, CreationContext context) {
		return new SendCMDProgramNodeContribution(apiProvider, view, model);
	}
}