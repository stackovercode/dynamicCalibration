package com.spin.dynamicCalibration.impl;

import org.osgi.framework.BundleActivator;
import org.osgi.framework.BundleContext;

import com.ur.urcap.api.contribution.ProgramNodeService;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;

import com.ur.urcap.api.contribution.InstallationNodeService;


public class Activator implements BundleActivator {
	@Override
	public void start(BundleContext bundleContext) throws Exception {
		System.out.println("Activator starts!");
		// Message
		bundleContext.registerService(SwingProgramNodeService.class, new SendCMDProgramNodeService(), null);

			
		
		// Server setup & close
		bundleContext.registerService(SwingProgramNodeService.class, new ServerSetupProgramNodeService(), null);
		bundleContext.registerService(ProgramNodeService.class, new ServerShutdownProgramNodeService(), null);
	}

	@Override
	public void stop(BundleContext bundleContext) throws Exception {
		System.out.println("Ending activator!");
	}
}

